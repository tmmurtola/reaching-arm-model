/******************************

*******************************/


#pragma once

#include "stdio.h"
#include <string>


//---------------------------------- mjControl ---------------------------------------------

struct _mjControl
{

    // simulation parameters
    double sim_duration = 4.0;
    double error_tol = 0.001;

    // loop control variables
    double error_norm = 1.0;
    mjtNum cum_error_res = 0.0;
    mjtNum cum_error_d = 0.0;

    // target sequence control
    int ntargs = 4;                    
    int next_targ = 0;                  // counter, set to 0 unless first targets need to be skipped
    double targ_list[8] = { 0.1573, 0.7464, -0.5496, 0.4745, -0.06021, 0.2026, 0.4292, 0.2570 };

    // --------- geometric variables --------

    bool use_joint[3] = { 1, 1, 1 };
    double init_pose[3] = { 20, 80.0, 20.0 };
    mjtNum warmstart[3] = { -6.39087667, 25.64324020, -9.42821378 };
    double shoulder_pos[3] = { 0.0, 0.0, 0.0 };
    double nominal_moment_arms[6] = { -0.045, 0.045, -0.041, 0.041, -0.035, 0.035 };
    double max_reach;
    double max_reach_elbow;
    double min_reach;
    double min_reach_elbow;
    double workspace_miny = 0.1;

    // --------- force variables ------------
    mjtNum* gain_len;
    mjtNum* gain_vel;
    //mjtNum* time_since_fired;
    //double last_firing_rate[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    


    // --------- task control ---------------

    double K[9] = {79418.58, 1302.5962, 0, 86022.3, 1704.9272, 0, 66822.68, 1992.8346, 0};

    double delay = 52;                    // prediction time in time steps
    double ap_dur = 0.003;                  // duration of excitation impulse
    double fire_reset = 1.0;                // time after decrecruitment when time_since_fired counter resets
    double reach_time_const = 1.712;        // default K^1/6 * 60^1/3, K=0.007 -> 1.712

    double reach_time;

    // target vectors
    mjtNum targ_xpos[3];            // target position in Cartesian coordinates

    // error vectors
    mjtNum x_error[3];               // position error (wrt target) in cartesian position
    mjtNum x_error_init[3];          // position error of initial pose
    mjtNum x_error_d[3];             // position error wrt desired position
    mjtNum x_error_pred[3];          // predicted position error wrt target

    mjtNum* des_qvel;
    mjtNum des_xvel[3];
    mjtNum* q_error;
    mjtNum* qvel_error;

    // Jacobians
    mjtNum* Jcb;
    mjtNum* Jpl;

    // torques & forces
    mjtNum* PDmagn;
    mjtNum* computed_torque;
    mjtNum* constr_torq;
    
     

    void init(const int nv, const int nu)
    {

        gain_len = new mjtNum[nu];
        gain_vel = new mjtNum[nu];
        //time_since_fired = new mjtNum[nu];
        //std::fill(time_since_fired, time_since_fired + nu, 0.0);

        des_qvel = new mjtNum[nv];
        q_error = new mjtNum[nv];
        qvel_error = new mjtNum[nv];

        Jcb = new mjtNum[3 * nv];
        Jpl = new mjtNum[nv * 3];
       
        PDmagn = new mjtNum[nv];
        computed_torque = new mjtNum[nv];
        constr_torq = new mjtNum[nv];

    }

    void del()
    {

        delete[] gain_len;
        delete[] gain_vel;
        //delete[] time_since_fired;
        
        delete[] des_qvel;
        delete[] q_error;
        delete[] qvel_error;

        delete[] Jcb;
        delete[] Jpl;

        delete[] PDmagn;
        delete[] computed_torque;
        delete[] constr_torq;

    }
};
typedef struct _mjControl mjControl;

struct _logControl
{
    bool time = 1;
    bool qpos = 1;
    bool qvel = 1;
    bool qacc = 0;

    bool targ_xpos = 1;
    bool grip_xpos = 1;

    bool ctrl = 1;
    bool PD = 1;
    bool computed_torque = 1;
    bool actuator_force = 1;
    bool actuator_moment = 0;
    bool actuator_length = 0;
    bool actuator_velocity = 0;
    bool gain_len = 0;
    bool gain_vel = 0;
    bool act = 1;
    bool act_dot = 0;
    bool userdata = 0;

    bool qfrc_actuator = 0;
    bool qfrc_applied = 0;
    bool qfrc_inverse = 0;
    
    bool xanchor = 0;

    std::string separator = ";\t";
 
    bool metaIsDone = 0;
};
typedef struct _logControl logControl;
