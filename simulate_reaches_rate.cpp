/***********************************************************************************
    SIMULATE_REACHES_RATE.CPP and associated header files run a set of upper arm reaching
    simulations with rate coded muscles using MuJoCo. For guidance on usage, see

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


#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "mjcontrol.h"
#include "reachingToolbox.h"


// file input/output
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include "ioToolbox.h"


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* dinit = NULL;               // MuJoCo data
mjData* d = NULL;                   
mjData* dpred = NULL;
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

mjControl c;
std::vector<std::pair<double, double>> targ_list;
logControl L;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    int trgid = mj_name2id(m, mjOBJ_SITE, "target");   // site for visualisation only
    int grpid = mj_name2id(m, mjOBJ_SITE, "grip");


    // backspace: reset simulations
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_copyData(d, m, dinit);
        mj_copyData(dpred, m, d);
        c.next_targ = 0;
        double temp_targ_xpos[2] = { targ_list[c.next_targ].first, targ_list[c.next_targ].second };
        nextTarget(m, d, temp_targ_xpos, c);
        c.cum_error_d = 0.0;
        c.cum_error_res = 0.0;
    }

    // space: next simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_SPACE)
    {
        mj_copyData(d, m, dinit);
        mj_copyData(dpred, m, d);
        c.next_targ++;
        if (c.next_targ < c.ntargs)
        {
            double temp_targ_xpos[2] = { targ_list[c.next_targ].first, targ_list[c.next_targ].second };
            nextTarget(m, d, temp_targ_xpos, c);
        }
        c.cum_error_d = 0.0;
        c.cum_error_res = 0.0;
    }

    // tab: reset with new random target
    if (act == GLFW_PRESS && key == GLFW_KEY_TAB)
    {
        cam.distance = 0.90*cam.distance;
        cam.elevation = -90.0;
        cam.lookat[0] = 0.2087;
        cam.lookat[1] = 0.7011;
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}



// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc<2 )
    {
        printf(" USAGE: reach_sequence [modelfile] O[outputfile] T[targetfile] C[controlfile]\n");
        return 0;
    }

    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    //m->nuserdata = 3 * m->na;   // used to store muscle activation states
    int nact = m->nuserdata / m->nu;
    c.init(m->nv, m->nu);

    // make data
    dinit = mj_makeData(m);              // initial reference state
    setInitialPose(m, dinit, c.init_pose);
    optimalLengthFromCurrent(m, dinit, c);
    mju_copy3(dinit->qacc_warmstart, c.warmstart);
   
    d = mj_makeData(m);
    dpred = mj_makeData(m);


    // -- set muscle dynamics callbacks
    mjcb_act_gain = muscleGainFLV;
    mjcb_act_dyn = muscleActivation3rdOrder;
    mjcb_act_bias = muscleBias;

    // -- open output file
    std::ofstream outfile;
    std::string metafilename;
    bool use_target_file = false;

    for (int argind = 2;  argind < argc; argind++)
    {

        std::string argstr(argv[argind]);
        char typemarker(argstr.at(0));
        if (typemarker == 'O')  // output file provided
        {
            argstr.erase(argstr.begin()+0); // remove type marker
            std::cout << "Opening " << argstr << " for output\n";
            outfile.open(argstr);
            metafilename = argstr;
            metafilename.replace(metafilename.find(".txt"), 4, ".meta");
            std::cout << "Metadata will be stored in " << metafilename << "\n";

            if (!outfile.is_open())
            {
                mju_error("Failed to open the output file.");
            }
        }

        if (typemarker == 'T') // target file provided
        {
            use_target_file = true;
            std::ifstream targetfile;
            argstr.erase(argstr.begin() + 0); // remove type marker
            std::cout << "Importing targets from " << argstr << "\n";
            targetfile.open(argstr);

            if (!targetfile.is_open())
            {
                mju_error("Failed to open the target file.");
            }

            importTargets(targetfile, targ_list, c);
            targetfile.close();
        }

        
        if (typemarker == 'C') // control parameter file provided
        {
            std::ifstream ctrlfile;
            argstr.erase(argstr.begin() + 0); // remove type marker
            std::cout << "Importing control parameters from " << argstr << "\n";
            ctrlfile.open(argstr);

            if (!ctrlfile.is_open())
            {
                mju_error("Failed to open the control parameter file.");
            }

            importScaledControlParameters(ctrlfile, c);
            ctrlfile.close();
        }

    }

    if (!use_target_file)
        readTargetSequence(4, targ_list, c);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    cam.elevation = -90.0;

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    bool cont2next;
    int Nsteps, Nplan;
    bool use_predicted_data;
    
   

    // run main loop, target real-time simulation and 60 fps rendering
    while(!glfwWindowShouldClose(window))
    {
        while (c.next_targ < c.ntargs)  // loop through targets
        {
            
            // initialise data
            mj_copyData(d, m, dinit);
            for (int i = 0; i < m->nu; i++)
                d->userdata[i * nact + 2] = 10.0 * c.sim_duration;  //time_since_fired = long
            mj_copyData(dpred, m, d);

            //set target location
            double temp_targ_xpos[2] = { targ_list[c.next_targ].first, targ_list[c.next_targ].second };
            nextTarget(m, d, temp_targ_xpos, c);

            // initialise performance errors
            c.cum_error_d = 0.0;
            c.cum_error_res = 0.0;

            use_predicted_data = ((int)c.delay > 0);

            while (d->time < 1.5*c.reach_time)  // simulate until desired simulation duration
            {
                mjtNum simstart = d->time;
                while (d->time - simstart < 1.0 / 60.0)  // step simulation until time to render
                {
                    // log current state before advacing simulation
                    if (outfile.is_open())
                        logGeneral(outfile, d, m, c, L, metafilename);

                    // prediction loop
                    if (use_predicted_data)
                    {
                            
                        for (int i = 0; i < (int)c.delay; i++)
                        {
                            // handle excitation impulses (rate coding) during prediction
                            for (int actID = 0; actID < m->nu; actID++)
                            {
                                // default: dpred->ctrl is inherited from d->ctrl and kept constant
                                if (dpred->ctrl[actID] > 0.0 && dpred->userdata[actID * nact + 3] > c.ap_dur)  // excitation impuse completed, switch it off
                                    dpred->ctrl[actID] = 0.0;
                                else if (dpred->ctrl[actID] > 0.0)  // ongoing excitation impulse, update its duration
                                    dpred->userdata[actID * nact + 3] += m->opt.timestep;                               
                            }
                            // step prediction
                            mj_step(m, dpred);

                        }
                    }

                    // step main simulation
                    mj_step1(m, d);
                    muscleControlTargetCompTorqRate(m, d, dpred, use_predicted_data, c);
                    mj_step2(m, d);


                    updateErrors(m, d, use_predicted_data, c);


                    // copy simulation state
                    dpred->time = d->time;
                    mju_copy(dpred->qpos, d->qpos, m->nq);
                    mju_copy(dpred->qvel, d->qvel, m->nv);
                    mju_copy(dpred->act, d->act, m->na);

                    // copy userdata & control
                    mju_copy(dpred->userdata, d->userdata, m->nuserdata);
                    mju_copy(dpred->ctrl, d->ctrl, m->nu);
                    
                    // copy warm-start acceleration
                    mju_copy(dpred->qacc_warmstart, d->qacc_warmstart, m->nv);

                }

                // do rendering
                // get framebuffer viewport
                mjrRect viewport = { 0, 0, 0, 0 };
                glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

                // update scene and render
                mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
                mjr_render(viewport, &scn, &con);

                // swap OpenGL buffers (blocking call due to v-sync)
                glfwSwapBuffers(window);

                // process pending GUI events, call GLFW callbacks
                glfwPollEvents();
            }

            // simulation for target completed, summarise and move to next
            Nsteps = (int)(d->time / m->opt.timestep) + 1;
            Nplan = (int)(c.reach_time / m->opt.timestep) + 1;

            printf("\nTarget number %i: Finished at simulation at %f sec with stabilisation error %f and movement error %f.", c.next_targ, d->time,
                c.cum_error_res / (Nsteps - Nplan) / c.error_tol, c.cum_error_d / Nsteps / c.error_tol);

            c.next_targ++;
        }

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }
 
    c.del();

    // close output file
    outfile.close();

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
