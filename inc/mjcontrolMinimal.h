/******************************

*******************************/


#pragma once

#include "stdio.h"


//---------------------------------- mjControl ---------------------------------------------

struct _mjControl
{

    // --------- task control ---------------


    // -- FVL + 3rd ORD
    //double K[9] = { 5756.4, 3.2152 * sqrt(5756.4), 0, 296.84, 0.783 * sqrt(296.84), 0, 689.64, 0.1253 * sqrt(689.64), 0 };
    //double K[9] = { 1307.98, 2.7052 * sqrt(1307.98), 0, 459.45, 0.7898 * sqrt(459.45), 0, 71.54, 0.4971 * sqrt(71.54), 0 };				
    
    //double K[9] = { 10280.01, 3.2777 * sqrt(10280.01), 0, 200.59, 0.7847 * sqrt(200.59), 0, 691.2, 0.1253 * sqrt(691.2), 0 };
    //double K[9] = { 1333.74, 2.7677 * sqrt(1333.74), 0, 321.85, 0.9965 * sqrt(321.85), 0, 850.58, 0.1253 * sqrt(850.58), 0 };

    //double K[9] = { 192.97, 1.3909 * sqrt(192.97), 0, 671.47, 0.2971 * sqrt(671.47), 0, 289.37, 0.126 * sqrt(289.37), 0 };
    //double K[9] = { 176.51, 1.3133 * sqrt(176.51), 0, 573.55, 0.2328 * sqrt(573.55), 0, 370.0, 0.0642 * sqrt(370.0), 0 };
    //double K[9] = { 128.44, 1.4132 * sqrt(128.44), 0, 447.47, 0.2762 * sqrt(447.47), 0, 229.05, 0.0967 * sqrt(229.05), 0 };

    //double K[9] = { 892.22, 0.7070 * sqrt(892.22), 0, 402.75, 0.0824 * sqrt(402.75), 0,  670.34, 0.0323 * sqrt(670.34), 0 };
    //double K[9] = { 5632.25, 1.3327 * sqrt(5632.25), 0, 2746.16, 0.1944 * sqrt(2746.16), 0,  298.98, 0.0513 * sqrt(298.98), 0 };
    //double K[9] = { 4713.42, 0.7755 * sqrt(4713.42), 0, 1725.92, 0.0174 * sqrt(1725.92), 0,  900.87, 0.0068 * sqrt(900.87), 0 };
    //double K[9] = { 4766.28, 0.6971 * sqrt(4766.28), 0, 1699.28, 0.0155 * sqrt(1699.28), 0,  861.22, 0.0053 * sqrt(861.22), 0 };
    //double K[9] = { 4584.57, 0.7944 * sqrt(4584.57), 0, 1666.58, 0.0227 * sqrt(1666.58), 0,  919.7, 0.0066 * sqrt(919.7), 0 };
    //double K[9] = { 5969.22, 0.6229 * sqrt(5969.22), 0, 1695.24, 0.0075 * sqrt(1695.24), 0,  936.67, 0.0037 * sqrt(936.67), 0 };
    //double K[9] = { 899.36, 0.9042 * sqrt(899.36), 0, 462.41, 0.1131 * sqrt(462.41), 0,  761.3, 0.0223 * sqrt(761.3), 0 };
    //double K[9] = { 5059.59, 0.7112 * sqrt(5059.59), 0, 1743.43, 0.0205 * sqrt(1743.43), 0,  938.89, 0.0019 * sqrt(938.89), 0 };
    //double K[9] = { 1634.37, 1.6868 * sqrt(1634.37), 0, 1598.33, 1.6840 * sqrt(1598.33), 0,  1146.1, 2.0282 * sqrt(1146.1), 0 };
    //double K[9] = { 86360.0, 7.2 * sqrt(86360.0), 0, 129330.0, 7.2 * sqrt(129330.0), 0,  117160.0, 8.8 * sqrt(117160.0), 0 };
    //double K[9] = { 99870.0, 5.0 * sqrt(99870.0), 0, 116320.0, 4.1 * sqrt(116320.0), 0,  137740.0, 4.7 * sqrt(137740.0), 0 };
    //double K[9] = { 33314.23, 3.2242 * sqrt(33314.23), 0, 39365.59, 2.7212 * sqrt(39365.59), 0, 137485.18, 1.1785 * sqrt(137485.18), 0 };
    //double K[9] = { 22108.05, 700.6762, 0, 35379.61, 556.3841, 0, 48052.76, 459.989, 0 };
    double K[9] = { 38009.73, 259.2004, 0, 53285.75, 265.578, 0, 35575.89, 1040.989, 0 };
  
    double delay = 66;                    // prediction time in time steps
    double ap_dur = 0.003;                  // duration of excitation impulse
    double fire_reset = 1.0;                // time after decrecruitment when time_since_fired counter resets
    double reach_time_const = 1.712;

    bool use_joint[3] = { 1, 1, 1 };
    //double last_firing_rate[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // target
    mjtNum targ_xpos[3];            // target position in Cartesian coordinates
    double reach_time;

    // error vectors
    mjtNum x_error[3];               // position error (wrt target) in cartesian position
    mjtNum x_error_init[3];          // position error of initial pose
    mjtNum x_error_d[3];             // position error wrt desired position

    // cumulative errors
    mjtNum cum_error_res = 0.0;
    mjtNum cum_error_d = 0.0;

};
typedef struct _mjControl mjControl;
