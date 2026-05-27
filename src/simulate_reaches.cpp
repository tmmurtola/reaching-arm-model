/***********************************************************************************  
    SIMULATE_REACHES.CPP and associated header files run a set of upper arm reaching 
    simulations using MuJoCo. For guidance on usage, see

        README.md

    Copyright 2021 Tiina Murtola/Royal Veterinary College


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


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjData* dpred = NULL;               // MuJoCo data for forward model
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

mjControl c;                        // control parameters and variables

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;



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
    if( argc<1 )
    {
        printf(" Specify modelfile by calling: simulate_reaches modelfile\n");
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

    m->nuserdata = 3 * m->na;   // used to store muscle activation states

    // make & initialise data and control structure
    d = mj_makeData(m);
    c.init(m->nv);

    // identify relevant sites
    int trgid = mj_name2id(m, mjOBJ_SITE, "target");   // site for target
    int grpid = mj_name2id(m, mjOBJ_SITE, "grip");     // site to be tracked
    int chestid = mj_name2id(m, mjOBJ_BODY, "chest");  // chest body id (for shoulder pos)

    // store position of the shoulder (origin of relative coordinates)
    c.shoulder_pos[0] = m->body_pos[3 * chestid];
    c.shoulder_pos[1] = m->body_pos[3 * chestid + 1];
    c.shoulder_pos[2] = m->body_pos[3 * chestid + 2];


    // -- set starting position & compute optimal muscle lengths
    setInitialPose(m, d, c.initPose);
    optimalLengthFromCurrent(m, d);

    // -- set up first target
    c.next_targ = 0;
    bool cont2next = nextTarget(m, d, trgid);

    // -- initialise activation states (in userdata)
    mju_zero(d->userdata, m->nuserdata);

    // -- make data for forward model
    dpred = mj_makeData(m);
    mj_copyData(dpred, m, d);

    // -- set muscle dynamics callbacks
    mjcb_control = muscleControlTargetPD;
    mjcb_act_gain = muscleGainFLV;
    //mjcb_act_gain = muscleGainConst;
    mjcb_act_dyn = muscleActivation3rdOrder;
    //mjcb_act_dyn = muscleActivation1stOrder;


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

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);


    // run main loop, target real-time simulation and 60 fps rendering
    while(!glfwWindowShouldClose(window))
    {
        if (d->time < c.sim_duration && c.next_targ < c.ntargs)
        {
            mjtNum simstart = d->time;
            while (d->time - simstart < 1.0 / 60.0)
            {
                if ((int)c.delay > 0)
                {
                    c.use_predicted_data = 0;
                    c.update_control = 0;
                    for (int i = 0; i < (int)c.delay; i++)
                    {
                        mj_step(m, dpred);
                    }
                    c.use_predicted_data = 1;
                    c.update_control = 1;
                    mju_copy(c.predicted_qvel, dpred->qvel, m->nv);
                    c.predicted_xpos[0] = dpred->site_xpos[3 * grpid];
                    c.predicted_xpos[1] = dpred->site_xpos[3 * grpid + 1];
                    c.predicted_xpos[2] = 0;
                }
                else
                {
                    c.use_predicted_data = 0;
                    c.update_control = 1;
                }

                mj_step(m, d);

                updateErrors(m, d);

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
        }
        else if (cont2next)
        {
            int Nsteps = (int)(d->time / m->opt.timestep) + 1;
            int Nsucc;
            if (c.time_succ < 0)
                Nsucc = Nsteps;
            else
            {
                Nsucc = Nsteps - (int)(c.time_succ / m->opt.timestep);
            }
            printf("\nTarget number %i: Finished at simulation with stabilisation error %f and movement error %f.", c.next_targ,
                c.cum_error_res / Nsucc / c.error_tol, c.cum_error_d / Nsteps / c.error_tol);
            c.next_targ++;
            if (c.next_targ < c.ntargs)
            {
                mj_resetData(m, d);
                setInitialPose(m, d, c.initPose);
                cont2next = nextTarget(m, d, trgid);
            }
            else
                cont2next = 0;

            if (cont2next)
            {
                // reset activation and control data
                mju_zero(d->userdata, m->nuserdata);
                mju_zero(d->ctrl, m->nu);
                mju_zero(d->act_dot, m->na);
                mj_copyData(dpred, m, d);
                mju_zero(dpred->userdata, m->nuserdata);
                c.error_norm = 1.0;
                c.cum_error_d = 0.0;
                c.cum_error_res = 0.0;
                c.success = 0;
                c.time_succ = -1.0;
            }
            else
            {
                c.error_norm = 0.0;
                printf("\n No more targets.");
            }
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

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free data, deactivate
    c.del();
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    return 1;
}
