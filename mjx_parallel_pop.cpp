/***********************************************************************************
    MJX_PARALLEL_POP.CPP can be used to create a mex file for running reaching simulations
    on MATLAB. Core model mirrors SIMULATE_REACHES_RATE.CPP. The code allows for parallellisation
    of multiple simulations either by control parameter or by target.
    
    For guidance on usage, see

        README.md

    Copyright 2025 Tiina Murtola/Royal Veterinary College


    Parts of this code are modified from MuJoCo Resources, under the MuJoCo
    Resource License:

        "Copyright 2018, Roboti LLC

        This file is licensed under the MuJoCo Resource License (the "License").
        You may not use this file except in compliance with the License.
        You may obtain a copy of the License at

            https://www.roboti.us/resourcelicense.txt"


 **********************************************************************************/



#include "mex.h"
#include "mujoco.h"
#include "mjxmacro.h"
#include "glfw3.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "mjcontrolMinimal.h"   // minmal data set to be stored
#include "reachingToolbox.h"
#include <omp.h>
#include <vector>

using namespace std;

//--------------------------- basic.cpp with thread/mutex -------------------------------

bool initialized = false;
bool exitrequest = false;

const int max_threads = 40;
int nthread;

// main data structures: protected with mutex
mjModel* m = NULL;
mjData* dinit = NULL;
mjData* d[max_threads];
mjData* dpred[max_threads];
mjControl c[max_threads];


struct Controls
{
    double K[6];
    double delay;
};

vector<pair<double, double>> targ_list;
int ntargs;
vector<Controls> population;
int npop;

int nact;


//--------------------------- MATLAB-specific code --------------------------------------

// error
void mju_MATLAB_error(const char* msg)
{
    mexErrMsgTxt(msg);
}


// warning
void mju_MATLAB_warning(const char* msg)
{
    //mexWarnMsgTxt(msg);
}


// allocate memory with mxMalloc, make persistent
void* mju_MATLAB_malloc(size_t sz)
{
    void* ptr = mxMalloc(sz);
    mexMakeMemoryPersistent(ptr);
    return ptr;
}


// free memory with mxFree
void mju_MATLAB_free(void* ptr)
{
    mxFree(ptr);
}

void exitFunction(void)
{
    initialized = false;
    exitrequest = false;

    // unlock mex so MATLAB can remove it from memory
    mexUnlock();
}

// help string
static char helpstr[] = 
"USAGE:\n"
"  [result] = mjx_render(command, [parameters]);\n"
"LOADING:\n"
"  mjx('load', '../model/human_amr_3dof_moments.xml');\n"
"COMMANDS AND PARAMETERS:\n"
"  simulate             : simulate to all targets with parallel population, return average errors\n"
"  simulate_par_targs   : simulate to all targets in parallel, return average errors\n"
"  reset                : reset simulation and target sequence\n"
"  get_x_error          : returns 1x3 error in endpoint position relative to target\n"
"  get_cumulative_errors : total stabilisation and movement errors (1x2) so far\n"
"  get_average_errors   : average stabilisation and movement errors (1x2) so far\n"
"  get_target_list      : returns list of target locations (2xn) \n"
"  set_target_list target_array : set target locations to target_array (2xn), does not reset target counter \n"
"  get_K                : returns 3x3 feedback gain matrix \n"
"  set_K gain_array     : set 3x3 feedback gain matrix to gain_array \n" 
"  get_delay            : returns prediction time of forward model \n"
"  set_delay delay      : set prediction time to delay \n"
"EXIT:\n"
"  exit or quit   : terminate\n\n";


// entry point
void mexFunction(int nout, mxArray* pout[], int nin, const mxArray* pin[])
{
    char filename[100], command[100], fieldname[100];

    double sim_duration = 4.0;
    double error_tol = 0.001;

    // no inputs: print help, return
    if( !nin )
    {
        mexPrintf(helpstr);
        return;
    }

    // get command string
    if( mxGetClassID(pin[0])!=mxCHAR_CLASS )
    {
        mexPrintf("first argument must be command string\n");
        return;
    }
    mxGetString(pin[0], command, 100);

    //---------------------------- initialize and load model file
    if( !strcmp(command, "load") )
    {
        // check for repeated initialization
        if( initialized )
           mexErrMsgTxt("already initialized");

        // get filename
        if( nin<2 || mxGetClassID(pin[1])!=mxCHAR_CLASS )
           mexErrMsgTxt("second argument must be filename");
        mxGetString(pin[1], filename, 100);

        // set MATLAB handlers
        mju_user_error = mju_MATLAB_error;
        mju_user_warning = mju_MATLAB_warning;
        mju_user_malloc = mju_MATLAB_malloc;
        mju_user_free = mju_MATLAB_free;

        // activate MuJoCo
        mj_activate("mjkey.txt");

        // initialise target sequence
        ntargs = 4;
        double init_targ_list[8] = { 0.1573, 0.7464, -0.5496, 0.4745, -0.06021, 0.2026, 0.4292, 0.2570 };
        for (int n = 0; n < ntargs; n++)
            targ_list.push_back(make_pair(init_targ_list[n * 2], init_targ_list[n * 2 + 1]));

        // initialise population vector from c[0]
        npop = 1;
        population.push_back(Controls());
        population[0].delay = c[0].delay;
        for (int j = 0; j < 3; j++)
        {
            population[0].K[2 * j] = c[0].K[3 * j];
            population[0].K[2 * j + 1] = c[0].K[3 * j + 1];
        }

        // load and compile model
        char error[1000] = "Could not load binary model";
        if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
            m = mj_loadModel(filename, 0);
        else
            m = mj_loadXML(filename, 0, error, 1000);
        if( !m )
            mju_error_s("Load model error: %s", error);

        // space for muscle activation states
        //m->nuserdata = 3 * m->na;
        nact = m->nuserdata / m->nu;

        //mexPrintf("nuserdata %i, nu %i, nact %i\n", m->nuserdata, m->nu, nact);

        nthread = omp_get_num_procs();
        mexPrintf("Available threads: %i, Max threads for data structures: %i\n", nthread, max_threads);
        if (nthread > max_threads)
            mexErrMsgTxt("Number of threads exceeds data structure initialisation.");

        dinit = mj_makeData(m);              // initial reference state
        // -- set starting position & compute optimal muscle lengths
        double initPose[3] = { 20, 80, 20 };
        mjtNum warmstart[3] = { -6.39087667, 25.64324020, -9.42821378 };
        setInitialPose(m, dinit, initPose);
        optimalLengthFromCurrent(m, dinit, c[0]); //initialise using c[0].ap_dur
        mju_copy3(dinit->qacc_warmstart, warmstart);


        // make data and update
        for (int n = 0; n < nthread; n++)
        {
            d[n] = mj_makeData(m);
            dpred[n] = mj_makeData(m);
        }

        // -- set muscle dynamics callbacks
        mjcb_act_gain = muscleGainFLV;
        mjcb_act_dyn = muscleActivation3rdOrder;
        mjcb_act_bias = muscleBias;

        // finish initialization
        mexAtExit(exitFunction);
        mexLock();
        initialized = true;

        mexPrintf("MJX initialised.\n");

        // start background processing
        exitrequest = false;
        return;
    }

    //---------------------------- terminate
    else if( !strcmp(command, "exit") || !strcmp(command, "quit") )
    {
        if (initialized)
        {
            // wait for thread to exit
            exitrequest = true;

            // delete model and data, deactivate
            for (int n = 0; n < nthread; n++)
            {
                mj_deleteData(d[n]);
                mj_deleteData(dpred[n]);            
                d[n] = NULL;
                dpred[n] = NULL;
            }
            mj_deleteModel(m);
            m = NULL;
            mj_deactivate();
    
            exitFunction();
        }
        return;
    }

    // the remaining commands require initialization
    if( !initialized )
        mexErrMsgTxt("not initialized");


    //---------------------------- get/set targ_list
    else if (!strcmp(command, "get_target_list"))
    {
        //lock_guard<mutex> guard(mtx);

        // find field
        int nr = ntargs, nc = 2;
        double* temp = new double[nr * 2];
        for (int n = 0; n < ntargs; n++)
        {
            temp[n * 2] = targ_list[n].first;
            temp[n * 2 + 1] = targ_list[n].second;
        }

        // create MATLAB matrix and copy data (assuming mjtNum is double)
        pout[0] = mxCreateDoubleMatrix(nc, nr, mxREAL);
        memcpy(mxGetPr(pout[0]), temp, nr * nc * sizeof(double));

    }

    else if (!strcmp(command, "set_target_list"))
    {
        //lock_guard<mutex> guard(mtx);

        
        int nc = 2;

        if (nin < 2 || !mxIsNumeric(pin[1]))
            mexErrMsgTxt("target array expected");

        // check data dimensions
        const mwSize* sz = mxGetDimensions(pin[1]);
        if (mxGetNumberOfDimensions(pin[1]) != 2 || sz[0] != nc)
            mexErrMsgTxt("invalid data dimensions");
        int nr = sz[1];

        // copy data (assuming mjtNum is double)
        double* temp = new double[nc * nr];
        memcpy(temp, mxGetPr(pin[1]), nr * nc * sizeof(double));


        // update number of targets and overwrite given elements of targ_list

        for (int n = 0; n < nr; n++)
        {
            if (n < ntargs)
            {
                targ_list[n].first = temp[n * 2];
                targ_list[n].second = temp[n * 2 + 1];
            }   
            else
                targ_list.push_back(make_pair(temp[n * 2], temp[n * 2 + 1]));

        };

        ntargs = nr;

        delete[] temp;

    }

    //---------------------------- get/set population
    else if (!strcmp(command, "get_population"))
    {
        //lock_guard<mutex> guard(mtx);

        // find field
        int nr = npop, nc = 7;
        double* temp = new double[nr * nc];
        for (int n = 0; n < npop; n++)
        {
            for (int j = 0; j < 6; j++)
                temp[n * nc + j] = population[n].K[j];
            temp[n * nc + 6] = population[n].delay;
        }

        // create MATLAB matrix and copy data (assuming mjtNum is double)
        pout[0] = mxCreateDoubleMatrix(nc, nr, mxREAL);
        memcpy(mxGetPr(pout[0]), temp, nr * nc * sizeof(double));

    }

    else if (!strcmp(command, "set_population"))
    {
        const int nc = 7;

        if (nin < 2 || !mxIsNumeric(pin[1]))
            mexErrMsgTxt("population array expected");

        // check data dimensions
        const mwSize* sz = mxGetDimensions(pin[1]);
        if (mxGetNumberOfDimensions(pin[1]) != 2 || sz[0] != nc)
            mexErrMsgTxt("invalid data dimensions");
        int nr = sz[1];

        // copy data (assuming mjtNum is double)
        double* temp = new double[nc * nr];
        memcpy(temp, mxGetPr(pin[1]), nr* nc * sizeof(double));

        for (int n = 0; n < nr; n++)
        {
            if (!(n < npop))
                population.push_back(Controls());

            for (int j = 0; j < 6; j++)
                population[n].K[j] = temp[n * nc + j];
            population[n].delay = temp[n * nc + 6];
        };

        npop = nr;
    }


    //---------------------------- get/set reach_time_const
    else if (!strcmp(command, "get_reach_time_const"))
    {

    // find field
    int nr = 1, nc = nthread;
    double* temp = new double[nr * nc];
    for (int n = 0; n < nr; n++)
        temp[n] = c[n].reach_time_const;


    // create MATLAB matrix and copy data (assuming mjtNum is double)
    pout[0] = mxCreateDoubleMatrix(nc, nr, mxREAL);
    memcpy(mxGetPr(pout[0]), temp, nr * nc * sizeof(double));

    }
    else if (!strcmp(command, "set_reach_time_const"))
    {

    if (nin < 2 || !mxIsNumeric(pin[1]))
        mexErrMsgTxt("single time constant expected");

    // check data dimensions
    int nr = 1, nc = 1;
    const mwSize* sz = mxGetDimensions(pin[1]);
    if (mxGetNumberOfDimensions(pin[1]) != 2 || sz[0] != nc || sz[1] != nr)
        mexErrMsgTxt("invalid data dimensions");

    // copy data (assuming mjtNum is double)
    double* temp = new double[nr * nc];
    memcpy(temp, mxGetPr(pin[1]), nr * nc * sizeof(double));
    for (int n = 0; n < nthread; n++)
        c[n].reach_time_const = temp[0];

    }

    //---------------------------- get control parameter list
    else if (!strcmp(command, "get_ctrl_params"))
    {
        int nc = 7, nr = nthread;
        double* temp = new double[nc * nr];

        for (int n = 0; n < nr; n++)
        {
            for (int j = 0; j < 3; j++)
            {
                temp[7 * n + 2 * j] = c[n].K[3 * j];
                temp[7 * n + 2 * j + 1] = c[n].K[3 * j + 1];
            }
            temp[7 * n + 6] = c[n].delay;
        }

        // create MATLAB matrix and copy data (assuming mjtNum is double)
        pout[0] = mxCreateDoubleMatrix(nc, nr, mxREAL);
        memcpy(mxGetPr(pout[0]), temp, nr * nc * sizeof(double));

        delete[] temp;
    }

    else if (!strcmp(command, "set_ctrl_params"))
    {

        if (nin < 2 || !mxIsNumeric(pin[1]))
          mexErrMsgTxt("control parameter vector expected");

        const int nc = 7;
        double temp[nc];
        double* fielddata = (double*)temp;

        // check data dimensions
        const mwSize* sz = mxGetDimensions(pin[1]);
        if (mxGetNumberOfDimensions(pin[1]) != 2 || sz[0] != 1 || sz[1] != nc)
            mexErrMsgTxt("invalid data dimensions");

        // copy data (assuming mjtNum is double)
        memcpy(temp, mxGetPr(pin[1]), nc * sizeof(double));

        for (int n = 0; n < nthread; n++)
        {
            for (int j = 0; j < 3; j++)
            {
                 c[n].K[3 * j] = temp[2 * j];
                 c[n].K[3 * j + 1]= temp[2 * j + 1] ;
            }
            c[n].delay = temp[6];
        }

    }

    //---------------------------- simulate (parallel population)
    else if( !strcmp(command, "simulate") )
    {
        //lock_guard<mutex> guard(mtx);

        mjtNum* errors = new mjtNum[npop * ntargs];

        #pragma omp parallel for
        for (int ipop = 0; ipop < npop; ipop++)
        {
            int thrID = omp_get_thread_num();
            for (int j = 0; j < 3; j++)
            {
                c[thrID].K[3 * j] = population[ipop].K[2 * j];
                c[thrID].K[3 * j + 1] = population[ipop].K[2 * j + 1];
            }
            c[thrID].delay = population[ipop].delay;

            for (int itarg = 0; itarg < ntargs; itarg++)
            {
                //mexPrintf("nuserdata %i, nu %i, nact %i", m->nuserdata, m->nu, nact);

                mj_copyData(d[thrID], m, dinit);
                for (int i = 0; i < m->nu; i++)
                    d[thrID]->userdata[i * nact + 2] = 10.0 * sim_duration;
                //mju_printMat(d[thrID]->userdata, m->nu, nact);
                mj_copyData(dpred[thrID], m, d[thrID]);
                //std::fill(c[thrID].last_firing_rate, c[thrID].last_firing_rate + m->nu, 0.0);
                double temp_targ_xpos[2] = { targ_list[itarg].first, targ_list[itarg].second };
                nextTarget(m, d[thrID], temp_targ_xpos, c[thrID]);

                c[thrID].cum_error_d = 0.0;
                c[thrID].cum_error_res = 0.0;
                bool use_pred_data = ((int)c[thrID].delay > 0);
                //double ctrltime = -1.0;

                while (d[thrID]->time < 1.5*c[thrID].reach_time)
                {
                    //if (d[thrID]->time - ctrltime >= 0.002)
                    //{
                        if (use_pred_data)
                        {
                            for (int i = 0; i < (int)c[thrID].delay; i++)
                            {
                                for (int actID = 0; actID < m->nu; actID++)
                                {
                                    if (dpred[thrID]->ctrl[actID] > 0.0 && dpred[thrID]->userdata[actID * nact + 3] > c[thrID].ap_dur)
                                        dpred[thrID]->ctrl[actID] = 0.0;
                                    else if (dpred[thrID]->ctrl[actID] > 0.0)
                                        dpred[thrID]->userdata[actID * nact + 3] += m->opt.timestep;
                                }
                                mj_step(m, dpred[thrID]);
                            }

                        }

                        mj_step1(m, d[thrID]);
                        //muscleControlTargetCompTorq(m, d[thrID], dpred[thrID], use_pred_data, c[thrID]);
                        muscleControlTargetCompTorqRate(m, d[thrID], dpred[thrID], use_pred_data, c[thrID]);
                        mj_step2(m, d[thrID]);
                        //ctrltime = d[thrID]->time;
                    //}
                    //else
                    //    mj_step(m, d[thrID]);

                    updateErrors(m, d[thrID], use_pred_data, c[thrID]);

                    // copy simulation state
                    dpred[thrID]->time = d[thrID]->time;
                    mju_copy(dpred[thrID]->qpos, d[thrID]->qpos, m->nq);
                    mju_copy(dpred[thrID]->qvel, d[thrID]->qvel, m->nv);
                    mju_copy(dpred[thrID]->act, d[thrID]->act, m->na);

                    // copy userdata & control
                    mju_copy(dpred[thrID]->userdata, d[thrID]->userdata, m->nuserdata);
                    mju_copy(dpred[thrID]->ctrl, d[thrID]->ctrl, m->nu);

                    // copy warm-start acceleration
                    mju_copy(dpred[thrID]->qacc_warmstart, d[thrID]->qacc_warmstart, m->nv);

                }

                int Nsteps = (int)(d[thrID]->time / m->opt.timestep) + 1;
                int Nplan = (int)(c[thrID].reach_time / m->opt.timestep) + 1;

                errors[ntargs * ipop + itarg] = c[thrID].cum_error_res / (Nsteps - Nplan) / error_tol;
                //mexPrintf("\nipop=%i npop=%i itarg=%i error=%f", ipop, npop, itarg, errors[ipop * npop + itarg]);
                //errors[2 * itarg + 1] = c[itarg].cum_error_d / Nsteps / error_tol;
            }
        }
        
        // create MATLAB matrix and copy data (assuming mjtNum is double)
        pout[0] = mxCreateDoubleMatrix(ntargs, npop, mxREAL);
        memcpy(mxGetPr(pout[0]), errors, npop * ntargs * sizeof(double));

        delete[] errors;
    }

    //---------------------------- simulate (parallel targets)
    else if (!strcmp(command, "simulate_par_targs"))
    {
        //lock_guard<mutex> guard(mtx);

        mjtNum* errors = new mjtNum[ntargs];

        #pragma omp parallel for
        for (int itarg = 0; itarg < ntargs; itarg++)
        {
            int thrID = omp_get_thread_num();

                mj_copyData(d[thrID], m, dinit);
                for (int i = 0; i < m->nu; i++)
                    d[thrID]->userdata[i * nact + 2] = 10.0 * sim_duration;
                mj_copyData(dpred[thrID], m, d[thrID]);
                //std::fill(c[thrID].last_firing_rate, c[thrID].last_firing_rate + m->nu, 0.0);
                double temp_targ_xpos[2] = { targ_list[itarg].first, targ_list[itarg].second };
                nextTarget(m, d[thrID], temp_targ_xpos, c[thrID]);

                c[thrID].cum_error_d = 0.0;
                c[thrID].cum_error_res = 0.0;
                bool use_pred_data = ((int)c[thrID].delay > 0);


                while (d[thrID]->time < 1.5 * c[thrID].reach_time)
                {
                    if (use_pred_data)
                    {
                        for (int i = 0; i < (int)c[thrID].delay; i++)
                        {
                            for (int actID = 0; actID < m->nu; actID++)
                            {
                                if (dpred[thrID]->ctrl[actID] > 0.0 && dpred[thrID]->userdata[actID * nact + 3] > c[thrID].ap_dur)
                                    dpred[thrID]->ctrl[actID] = 0.0;
                                else if (dpred[thrID]->ctrl[actID] > 0.0)
                                    dpred[thrID]->userdata[actID * nact + 3] += m->opt.timestep;
                            }
                            mj_step(m, dpred[thrID]);
                        }

                    }

                    mj_step1(m, d[thrID]);
                    //muscleControlTargetCompTorq(m, d[thrID], dpred[thrID], use_pred_data, c[thrID]);
                    muscleControlTargetCompTorqRate(m, d[thrID], dpred[thrID], use_pred_data, c[thrID]);
                    mj_step2(m, d[thrID]);

                    updateErrors(m, d[thrID], use_pred_data, c[thrID]);

                    // copy simulation state
                    dpred[thrID]->time = d[thrID]->time;
                    mju_copy(dpred[thrID]->qpos, d[thrID]->qpos, m->nq);
                    mju_copy(dpred[thrID]->qvel, d[thrID]->qvel, m->nv);
                    mju_copy(dpred[thrID]->act, d[thrID]->act, m->na);

                    // copy userdata & control
                    mju_copy(dpred[thrID]->userdata, d[thrID]->userdata, m->nuserdata);
                    mju_copy(dpred[thrID]->ctrl, d[thrID]->ctrl, m->nu);

                    // copy warm-start acceleration
                    mju_copy(dpred[thrID]->qacc_warmstart, d[thrID]->qacc_warmstart, m->nv);

                }

                int Nsteps = (int)(d[thrID]->time / m->opt.timestep) + 1;
                int Nplan = (int)(c[thrID].reach_time / m->opt.timestep) + 1;

                errors[itarg] = c[thrID].cum_error_res / (Nsteps - Nplan) / error_tol;
                //mexPrintf("\nipop=%i npop=%i itarg=%i error=%f", ipop, npop, itarg, errors[ipop * npop + itarg]);
                //errors[2 * itarg + 1] = c[itarg].cum_error_d / Nsteps / error_tol;
            
        }

        // create MATLAB matrix and copy data (assuming mjtNum is double)
        pout[0] = mxCreateDoubleMatrix(ntargs, npop, mxREAL);
        memcpy(mxGetPr(pout[0]), errors, npop * ntargs * sizeof(double));

        delete[] errors;
    }

    else if (!strcmp(command, "simulate_debug"))
    {
    //lock_guard<mutex> guard(mtx);

    mjtNum* errors = new mjtNum[npop * ntargs];

    //#pragma omp parallel for
    for (int ipop = 0; ipop < npop; ipop++)
    {
        int thrID = 1;
        for (int j = 0; j < 3; j++)
        {
            c[thrID].K[3 * j] = population[ipop].K[2 * j];
            c[thrID].K[3 * j + 1] = population[ipop].K[2 * j + 1];
        }
        c[thrID].delay = population[ipop].delay;

        mexPrintf("\n control parameters: Kp = [%f, %f, %f] Kv = [%f, %f, %f] tau = %f.", c[thrID].K[0], c[thrID].K[3], c[thrID].K[6], c[thrID].K[1], c[thrID].K[4], c[thrID].K[7], c[thrID].delay);

        for (int itarg = 0; itarg < ntargs; itarg++)
        {
            //mexPrintf("nuserdata %i, nu %i, nact %i", m->nuserdata, m->nu, nact);

            mj_copyData(d[thrID], m, dinit);
            for (int i = 0; i < m->nu; i++)
                d[thrID]->userdata[i * nact + 2] = 10.0 * sim_duration;
            //mju_printMat(d[thrID]->userdata, m->nu, nact);
            mj_copyData(dpred[thrID], m, d[thrID]);
            //std::fill(c[thrID].last_firing_rate, c[thrID].last_firing_rate + m->nu, 0.0);
            double temp_targ_xpos[2] = { targ_list[itarg].first, targ_list[itarg].second };
            nextTarget(m, d[thrID], temp_targ_xpos, c[thrID]);

            c[thrID].cum_error_d = 0.0;
            c[thrID].cum_error_res = 0.0;
            bool use_pred_data = ((int)c[thrID].delay > 0);
            //double ctrltime = -1.0;

            while (d[thrID]->time < 1.5 * c[thrID].reach_time)
            {
                if (use_pred_data)
                {
                    //if (d[thrID]->time < 0.01)
                    //    printf("\ntime: %.4f %.12f %.12f %.12f %.12f %.12f %.12f; ", d[thrID]->time, d[thrID]->qpos[0], d[thrID]->qpos[1], d[thrID]->qpos[2], d[thrID]->qvel[0], d[thrID]->qvel[1], d[thrID]->qvel[2]);
                    //{
                    //    printf("\ntime: %f, firing in main: ", d[thrID]->time);

                    //    for (int actID = 0; actID < m->nu; actID++)
                    //    {
                    //        if (d[thrID]->ctrl[actID] > 0.0)
                    //            printf("%i (%f/%f) ", actID, d[thrID]->userdata[actID * nact + 3], dpred[thrID]->userdata[actID * nact + 3]);
                    //    }
                    //}
                    for (int i = 0; i < (int)c[thrID].delay; i++)
                    {
                        for (int actID = 0; actID < m->nu; actID++)
                        {
                            if (dpred[thrID]->ctrl[actID] > 0.0 && dpred[thrID]->userdata[actID * nact + 3] > c[thrID].ap_dur)
                                dpred[thrID]->ctrl[actID] = 0.0;
                            else if (dpred[thrID]->ctrl[actID] > 0.0)
                                dpred[thrID]->userdata[actID * nact + 3] += m->opt.timestep;
                        }
                        mj_step(m, dpred[thrID]);
                        //if (d[thrID]->time < 0.01)
                        //    printf("\n pred_time: %.4f %.12f %.12f %.12f %.12f %.12f %.12f; ", dpred[thrID]->time, dpred[thrID]->qpos[0], dpred[thrID]->qpos[1], dpred[thrID]->qpos[2], dpred[thrID]->qvel[0], dpred[thrID]->qvel[1], dpred[thrID]->qvel[2]);
                        //{
                        //    printf("\npred_time: %f, firing in pred: ", dpred[thrID]->time);

                        //    for (int actID = 0; actID < m->nu; actID++)
                        //    {
                        //        if (dpred[thrID]->ctrl[actID] > 0.0)
                        //            printf("%i (%f)", actID, dpred[thrID]->userdata[actID * nact + 3]);
                        //    }
                        //}
                    }

                }
                //if (d[thrID]->time < 0.1)
                ////{
                //    mexPrintf("\n time=%.4f ", d[thrID]->time);
                //}

                mj_step1(m, d[thrID]);
                //muscleControlTargetCompTorq(m, d[thrID], dpred[thrID], use_pred_data, c[thrID]);
                muscleControlTargetCompTorqRate(m, d[thrID], dpred[thrID], use_pred_data, c[thrID]);
                mj_step2(m, d[thrID]);

                //mexPrintf("\n time=%f qpos = [%f, %f, %f] qvel = [%f, %f, %f]", d[thrID]->time, d[thrID]->qpos[0], d[thrID]->qpos[1], d[thrID]->qpos[2], d[thrID]->qvel[0], d[thrID]->qvel[1], d[thrID]->qvel[2]);
                //mexPrintf("\n time=%f firing: ", d[thrID]->time);
                //for (int actID = 0; actID < m->nu; actID++)
                //{
                //    if (d[thrID]->ctrl[actID] > 0.0)
                //        mexPrintf("%i ", actID);
                //}

                updateErrors(m, d[thrID], use_pred_data, c[thrID]);

                // copy simulation state
                dpred[thrID]->time = d[thrID]->time;
                mju_copy(dpred[thrID]->qpos, d[thrID]->qpos, m->nq);
                mju_copy(dpred[thrID]->qvel, d[thrID]->qvel, m->nv);
                mju_copy(dpred[thrID]->act, d[thrID]->act, m->na);

                // copy userdata & control
                mju_copy(dpred[thrID]->userdata, d[thrID]->userdata, m->nuserdata);
                mju_copy(dpred[thrID]->ctrl, d[thrID]->ctrl, m->nu);

                // copy warm-start acceleration
                mju_copy(dpred[thrID]->qacc_warmstart, d[thrID]->qacc_warmstart, m->nv);

            }

            int Nsteps = (int)(d[thrID]->time / m->opt.timestep) + 1;
            int Nplan = (int)(c[thrID].reach_time / m->opt.timestep) + 1;

            errors[ntargs * ipop + itarg] = c[thrID].cum_error_res / (Nsteps - Nplan) / error_tol;
            //mexPrintf("\nipop=%i npop=%i itarg=%i error=%f", ipop, npop, itarg, errors[ipop * npop + itarg]);
            //errors[2 * itarg + 1] = c[itarg].cum_error_d / Nsteps / error_tol;
        }
    }

    // create MATLAB matrix and copy data (assuming mjtNum is double)
    pout[0] = mxCreateDoubleMatrix(ntargs, npop, mxREAL);
    memcpy(mxGetPr(pout[0]), errors, npop * ntargs * sizeof(double));

    delete[] errors;
    }

    // undefined command
    else
        mexErrMsgTxt("undefined command string");
}
