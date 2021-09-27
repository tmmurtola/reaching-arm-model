/***************************************************************
    MJCONTROL.H defines a structure of control variables 
    for SIMULATE_REACHES.CPP.

    Copyright 2021 Tiina Murtola/Royal Veterinary College

***************************************************************/


#pragma once

#include "stdio.h"


//---------------------------------- mjControl ---------------------------------------------

struct _mjControl
{

    /* FREELY ADJUSTIBLE VARIABLES */

    // ---- simulation parameters ----
    double sim_duration = 4.0;
    double error_tol = 0.001;

    // ---- target sequence control ----
    int ntargs = 4;  
    int next_targ = 0;
    double targ_dist = 0.2175;
    double reach_time = 1.0;        // value overwritten in code

    // ---- control parameters ----

    // PD gains -- comment/uncomment or set manually as desired

    // -- FMAX
    //double K[6] = { 5582.22, 1.0816 * sqrt(5582.22), 2050.08, 0.0051 * sqrt(2050.08),  7541.4, 0.0090 * sqrt(7541.4) };   
    //double K[6] = { 4342.35, 1.0816 * sqrt(4342.35), 5154.67, 0.0051 * sqt(5154.67), 1622.29, 0.0090 * sqrt(1622.29) };

    // -- FMAX+O1
    //double K[6] = { 6920.38, 7.0322 * sqrt(6920.38), 328.13, 2.5314 * sqrt(328.13), 316.41, 0.3871 * sqrt(316.41) };
    //double K[6] = { 6919.67, 7.0008 * sqrt(6919.67), 325.00, 2.5313 * sqrt(325.00), 400.00, 0.3800 * sqrt(400.00) };

    // -- FVL+O1
    //double K[6] = { 7527.0, 6.3802 * sqrt(7527.0),  561.63, 2.3083 * sqrt(561.63), 223.45, 0.5001 * sqrt(223.45) };
    //double K[6] = { 7670.65, 6.2433 * sqrt(7670.65),  513.68, 2.498 * sqrt(513.68), 329.6, 0.4366 * sqrt(329.6) };

    // -- FMAX+O3				
    //double K[6] = { 38.33, 1.7957 * sqrt(38.33),  0.18, 0.5361 * sqrt(0.18), 2.96, 0.1417 * sqrt(2.96) };
    //double K[6] = { 67.89, 2.6026 * sqrt(67.89),  3.36, 0.9334 * sqrt(3.36), 15.03, 0.0895 * sqrt(15.03) };

    // -- FVL+O3
    double K[6] = { 192.97, 1.3909 * sqrt(192.97), 671.47, 0.2971 * sqrt(671.47), 289.37, 0.126 * sqrt(289.37) };
    //double K[6] = { 853.76, 1.3384 * sqrt(853.76), 743.07, 0.4673 * sqrt(743.07), 270.68, 0.105 * sqrt(270.68) };


    // predictin time (in time steps)
    double delay = 21;                    


    /* ADJUSTIBLE WITHIN XML GEOMETRY LIMITS */

    double initPose[3] = { 20, 80, 20 };
    double shoulder_pos[3] = { 0.0, 0.0, 0.0 };  // values overwritten in code



   
    /* KEY INITIALISATIONS - DO NOT TOUCH */

    // --------- force variables ------------
    mjtNum* gain_len;
    mjtNum* gain_vel;

    // loop control variables
    bool use_predicted_data = 0;
    bool update_control = 1;
    double error_norm = 1.0;
    mjtNum cum_error_res = 0.0;
    mjtNum cum_error_d = 0.0;

    // task variables
    bool success = 0;
    double time_succ = -1.0;

    // target vectors
    mjtNum targ_xpos[3];            // target position in Cartesian coordinates
    mjtNum targ_xvel[3];             // target velocity in Cartesian coordinates
    mjtNum* targ_qvel;   
    mjtNum* targ_qacc;

    // error vectors
    mjtNum x_error[3];               // position error (wrt target) in cartesian position
    mjtNum x_error_init[3];          // position error of initial pose
    mjtNum x_error_d[3];             // position error wrt desired position
    mjtNum* q_error;                 // position error in joint coordinates
    mjtNum* qvel_error;              // velocity error in joint coordinates


    // prediction
    mjtNum* predicted_qvel;
    mjtNum predicted_xpos[3];
    mjtNum x_error_pred[3];

    // Jacobians & related matrices
    mjtNum* Jcb;                   // end-effector Jacobian
    mjtNum* Jpl;                    // pseudo-inverse of jacp

    // torques & forces
    mjtNum* PDmagn;    
     

    void init(const int nv)
    {
        gain_len = new mjtNum[2 * nv];  // needs fixing if number of muslces is not 2x dofs
        gain_vel = new mjtNum[2 * nv];

        targ_qvel = new mjtNum[nv];     
        targ_qacc = new mjtNum[nv];

        q_error = new mjtNum[nv];
        qvel_error = new mjtNum[nv];

        predicted_qvel = new mjtNum[nv];

        Jcb = new mjtNum[3 * nv];
        Jpl = new mjtNum[nv * 3];
       
        PDmagn = new mjtNum[nv];
    }

    void del()
    {
        delete[] gain_len;
        delete[] gain_vel;

        delete[] targ_qvel;
        delete[] targ_qacc;

        delete[] q_error;
        delete[] qvel_error;

        delete[] predicted_qvel;

        delete[] Jcb;
        delete[] Jpl;

        delete[] PDmagn;
    }
};
typedef struct _mjControl mjControl;



