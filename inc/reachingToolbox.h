/*This toolbox contains the minimal set of functions to run reaching simulations
* 
*  
* 
* Created by Tiina Murtola
*/

#pragma once
#include <cmath>
#include "LinAlg.h"
#include <vector>


double FLgain(double l, double a, double b, double c)
{
    double Fgain;
    Fgain = exp(-pow(abs((pow(l, b) - 1) / c), a));

    return Fgain;
}

double FVgain(double v, double a, double b, double c, double vmax)
{
    double vnorm = -v / vmax;    //minus sign due to MuJoCo's sign convention
    double Fgain;

    if (vnorm > 0)
        Fgain = (1 - vnorm) / (1 + a * vnorm);
    else
        Fgain = b - (b - 1) * (1 + vnorm) / (1 - a * c * vnorm);

    return Fgain;
}


void optimalLengthFromCurrent(mjModel* m, const mjData* d, mjControl& c)
{
    /* Saves current length of actuator in m->actuator_user
    * for use as optimal muscle length in muscleBias and
    * muscleGain functions. Also calculates scaling factor
    * for activation when maximum firing rate is limited.
    */

    for (int actID = 0; actID < m->nu; actID++)
    {
        m->actuator_user[actID * m->nuser_actuator] = d->actuator_length[actID];

        double beta_tot = m->actuator_dynprm[actID * 10 + 1] * m->actuator_dynprm[actID * 10 + 3] * m->actuator_dynprm[actID * 10 + 5];
        double temp = 1.0 / (beta_tot / (m->actuator_user[actID * m->nuser_actuator + 2] * c.ap_dur) + 1.0 - beta_tot);
        m->actuator_user[actID * m->nuser_actuator + 5] = temp;  
    }
}


mjtNum muscleActivation3rdOrder(const mjModel* m, const mjData* d, int id)
{
    /* Computes and returns act_dot (for d->act_dot) for third order
    *  acivation dynamics. Intended to be use as activation callback
    *  in simulation pipeline. To use, set
    *    mjcb_act_dyn = muscleActivation3rdOrder;
    *  and check m->actuator_dynprm set in model file.
    */
    mjtNum act_dot;
    int nact = m->nuserdata / m->nu;

    d->userdata[nact * id] = d->userdata[nact * id] + m->opt.timestep / m->actuator_dynprm[id * 10 + 4] * d->ctrl[id]
        - m->opt.timestep / m->actuator_dynprm[id * 10 + 4] * (m->actuator_dynprm[id * 10 + 5]
            + (1 - m->actuator_dynprm[id * 10 + 5]) * d->ctrl[id]) * d->userdata[nact * id];

    d->userdata[nact * id + 1] = d->userdata[nact * id + 1] + m->opt.timestep / m->actuator_dynprm[id * 10 + 2] * d->userdata[nact * id]
        - m->opt.timestep / m->actuator_dynprm[id * 10 + 2] * (m->actuator_dynprm[id * 10 + 3]
            + (1 - m->actuator_dynprm[id * 10 + 3]) * d->userdata[nact * id]) * d->userdata[nact * id + 1];

    //d->userdata[3 * id + 2] = d->userdata[3 * id + 2] + m->opt.timestep / m->actuator_dynprm[id * 10] * d->userdata[3 * id + 1]
    //    - m->opt.timestep / m->actuator_dynprm[id * 10] * (m->actuator_dynprm[id * 10 + 1] 
    //        + (1 - m->actuator_dynprm[id * 10 + 1]) * d->userdata[3 * id + 1]) * d->userdata[3 * id + 2];

    act_dot = d->userdata[nact * id + 1] / m->actuator_dynprm[id * 10] - (m->actuator_dynprm[id * 10 + 1] + (1 - m->actuator_dynprm[id * 10 + 1]) * d->userdata[nact * id + 1]) * d->act[id] / m->actuator_dynprm[id * 10];

    return act_dot;
}

mjtNum muscleBias(const mjModel* m, const mjData* d, int id)
{
    /* Returns passive force from parallel elastic element.
       Intended to use as a muscle bias callback in simulation pipeline. To use, set
            mjcb_act_bias = muscleBias;
       and check m->actuator_biasprm.
    */

    mjtNum ForceBias;
    double delta_l_rel;

    delta_l_rel = d->actuator_length[id] / m->actuator_user[id * m->nuser_actuator] - m->actuator_biasprm[id * 10 + 2];

    double coco;

    if (id == 0 || id == 1)
        coco = 0.0 * 75.0;
    if (id == 2 || id == 3)
        coco = 0.0 * 50.0;
    if (id == 4 || id == 5)
        coco = 0.0 * 15.0;

    if (delta_l_rel > 0)
        ForceBias = m->actuator_biasprm[id * 10] * (exp(m->actuator_biasprm[id * 10 + 1] * delta_l_rel) - 1) - coco;
    else
        ForceBias = -coco;

    // correct for a_max
    ForceBias = ForceBias / m->actuator_user[id * m->nuser_actuator + 5];

    return ForceBias;
}

mjtNum muscleGainFLV(const mjModel* m, const mjData* d, int id)
{
    /* Returns muscle force from contractile element excluding the effects to dynamic activation.
       Intended to use as muscle gain callback in simulation pipeline. To use, set
            mjcb_act_gain = muscleGain;
       and check m->actuator_gainprm.
    */

    mjtNum ForceGain;

    // force gain from F-L curve
    mjtNum gain_len = FLgain((d->actuator_length[id]) / m->actuator_user[id * m->nuser_actuator], m->actuator_gainprm[id * 10 + 1],
        m->actuator_gainprm[id * 10 + 2], m->actuator_gainprm[id * 10 + 3]);

    // force gain from F-V curve
    mjtNum gain_vel = FVgain(d->actuator_velocity[id] / m->actuator_user[id * m->nuser_actuator], m->actuator_gainprm[id * 10 + 4],
        m->actuator_gainprm[id * 10 + 5], m->actuator_gainprm[id * 10 + 6],
        m->actuator_gainprm[id * 10 + 7]);   // velocity in the same units as s.vmax

    // total gain = Fmax*FVgain*FLgain (activation gain in a separate function)
    ForceGain = m->actuator_gainprm[id * 10] * gain_vel * gain_len;

    // correct for a_max
    ForceGain = ForceGain / m->actuator_user[id * m->nuser_actuator + 5];

    return ForceGain;
}

mjtNum muscleGainConst(const mjModel* m, const mjData* d, int id)
{
    /* Returns muscle force from contractile element excluding the effects to dynamic activation.
       Intended to use as muscle gain callback in simulation pipeline. To use, set
            mjcb_act_gain = muscleGain;
       and check m->actuator_gainprm.
    */

    mjtNum ForceGain;

    ForceGain = m->actuator_gainprm[id * 10];

    // correct for a_max
    ForceGain = ForceGain / m->actuator_user[id * m->nuser_actuator + 5];

    return ForceGain;
}



void nextTarget(mjModel* m, const mjData* d, const double targ_xpos[2], mjControl& c)
{
    int grpid = mj_name2id(m, mjOBJ_SITE, "grip");
 
    c.targ_xpos[0] = targ_xpos[0];
    c.targ_xpos[1] = targ_xpos[1];
    c.targ_xpos[2] = 0.0;

    c.x_error_init[0] = c.targ_xpos[0] - d->site_xpos[3 * grpid];
    c.x_error_init[1] = c.targ_xpos[1] - d->site_xpos[3 * grpid + 1];
    c.x_error_init[2] = 0.0;

    double targ_dist = sqrt(c.x_error_init[0] * c.x_error_init[0] + c.x_error_init[1] * c.x_error_init[1]);
    c.reach_time = c.reach_time_const * pow(targ_dist, 1.0 / 3.0);
}

void setInitialPose(const mjModel* m, mjData* d, double initPose[])
{
    for (int qID = 0; qID < m->nq; qID++)
        d->qpos[qID] = initPose[qID] / 180 * mjPI;
    mj_forward(m, d);
    mju_zero(d->qvel, m->nq);
    mju_zero(d->qacc, m->nv);
}

void updateErrors(const mjModel* m, mjData* d, const bool use_predicted_data, mjControl& c)
{
    int grpid = mj_name2id(m, mjOBJ_SITE, "grip");

    // -- position error in cartesian coordinates wrt target
    c.x_error[0] = c.targ_xpos[0] - d->site_xpos[3 * grpid];
    c.x_error[1] = c.targ_xpos[1] - d->site_xpos[3 * grpid + 1];
    c.x_error[2] = 0;  // 2D motion  

    double error_norm = mju_norm3(c.x_error);

    if (d->time > c.reach_time)
    {
        c.cum_error_res += error_norm;
    }

    double tau;
    double wk;
    mjtNum temp_error_d[3];

    if (!use_predicted_data)  // x_error_d corresponds to current timestep
        c.cum_error_d += mju_norm3(c.x_error_d);
    else if (d->time > c.reach_time) // wk=1, so error from desired = error from target
        c.cum_error_d += error_norm;
    else
    {
        tau = (d->time - m->opt.timestep) / c.reach_time; //time at time of xd computation
        wk = abs(15 * pow(tau, 4) - 6 * pow(tau, 5) - 10 * pow(tau, 3));

        mju_addScl3(temp_error_d, c.x_error, c.x_error_init, (wk - 1));
        c.cum_error_d += mju_norm3(temp_error_d);
    }
}


void psinv(mjtNum* res, mjtNum* J, const int n, const bool use_joint[])
{
    /* pseudo-inverse of J through SVD */

    alglib::real_2d_array a, u, vt, sinv, temp, ainv;
    alglib::real_1d_array w;

    int m = 3;
    int min_dim = std::min(m, n);

    //// Initialize arrays
    a.setlength(m, n);
    u.setlength(m, m);
    vt.setlength(n, n);
    w.setlength(min_dim);
    sinv.setlength(n, m);
    temp.setlength(n, m);
    ainv.setlength(n, m);

    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (use_joint[j] == 1)
                a[i][j] = J[i * n + j];
            else
                a[i][j] = 0.0;
            sinv[j][i] = 0.0;
        }
    }

    alglib::setglobalthreading(alglib::parallel);
    alglib::rmatrixsvd(a, m, n, 2, 2, 2, w, u, vt);

    for (int i = 0; i < min_dim; i++)
    {
        if (abs(w[i]) > 0.00001)
            sinv[i][i] = 1 / w[i];
        else
            sinv[i][i] = 0;
    }

    alglib::rmatrixgemm(n, m, n, 1.0, vt, 0, 0, 1, sinv, 0, 0, 0, 0.0, temp, 0, 0);
    alglib::rmatrixgemm(n, m, m, 1.0, temp, 0, 0, 0, u, 0, 0, 1, 0.0, ainv, 0, 0);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
            res[i * m + j] = ainv[i][j];
    }
}

void computePDmagn(const mjModel* m, const mjData* d, mjControl& c, mjtNum* res)
{
    // compute parameters for desired position and velocity
    double tau;
    mjtNum vk;
    mjtNum wk;

    tau = d->time / c.reach_time;
    if (tau > 1.0)
    {
        vk = 0.0;
        wk = 1.0;
    }
    else
    {
        vk = 1 / c.reach_time * abs(60 * pow(tau, 3) - 30 * pow(tau, 4) - 30 * pow(tau, 2));
        wk = abs(15 * pow(tau, 4) - 6 * pow(tau, 5) - 10 * pow(tau, 3));
    }

    int grpid = mj_name2id(m, mjOBJ_SITE, "grip");

    // -- position error in cartesian coordinates wrt target
    c.x_error[0] = c.targ_xpos[0] - d->site_xpos[3 * grpid];
    c.x_error[1] = c.targ_xpos[1] - d->site_xpos[3 * grpid + 1];
    c.x_error[2] = 0;  // 2D motion


    mjtNum* Jcb = new mjtNum[3 * m->nv];
    mjtNum* Jpl = new mjtNum[m->nv * 3];
    // -- compute the Jacobian & its pseudo-inverse
    mj_jacSite(m, d, Jcb, NULL, grpid);
    psinv(Jpl, Jcb, m->nv, c.use_joint);

    mjtNum* q_error = new mjtNum[m->nv];
    // -- compute positional and velocity errors in joint co-ordinates
    mju_addScl3(c.x_error_d, c.x_error, c.x_error_init, (wk - 1.0));
    mju_mulMatVec(q_error, Jpl, c.x_error_d, m->nv, 3);

    mjtNum* qvel_error = new mjtNum[m->nv];
    mjtNum des_xvel[3];
    mjtNum* des_qvel = new mjtNum[m->nv];
    mju_scl3(des_xvel, c.x_error_init, vk);
    mju_mulMatVec(des_qvel, Jpl, des_xvel, m->nv, 3);
    mju_sub(qvel_error, des_qvel, d->qvel, m->nv);

    // -- compute PD magnitude for each dof
    for (int qID = 0; qID < m->nv; qID++)
    {
        res[qID] = c.K[3 * qID] * q_error[qID] + c.K[3 * qID + 1] * qvel_error[qID]; // +c.K[3 * qID + 2] * q_error_int[qID];
    }
    delete[] Jpl;
    delete[] Jcb;
    delete[] q_error;
    delete[] qvel_error;
    delete[] des_qvel;
}

double desiredForce2Rate(const mjModel* m, double force_d, double FVLgain, int actID)
{
    double stim_drive = abs(force_d / FVLgain);
    double firing_rate = 0.0;

    int mupooltype = m->numeric_data[m->numeric_adr[mj_name2id(m, mjOBJ_NUMERIC, "mupool")]];

    if (mupooltype == 0) // exponential-linear
    {
        double min_rate = m->actuator_user[actID * m->nuser_actuator + 1];
        double max_rate = m->actuator_user[actID * m->nuser_actuator + 2];
        double min_recr = m->actuator_user[actID * m->nuser_actuator + 3];
        double max_recr = m->actuator_user[actID * m->nuser_actuator + 4];

        double coeff = (max_rate - min_rate) / (max_recr - min_recr);

        if (stim_drive < min_recr)
            firing_rate = 0.0;
        else if (stim_drive > max_recr)
            firing_rate = max_rate;
        else
            firing_rate = min_rate + coeff * (stim_drive - min_recr);
    }
    else if (mupooltype == 1) // mixed-logarithmic
    {
        double max_rate = m->actuator_user[actID * m->nuser_actuator + 2];
        double min_recr = m->actuator_user[actID * m->nuser_actuator + 3];
        double ref_drive = m->actuator_user[actID * m->nuser_actuator + 4];

        if (stim_drive <= min_recr)
            firing_rate = 0.0;
        else
        {
            firing_rate = 1 / (1 / max_rate - ref_drive / min_recr * log(1 - min_recr / stim_drive));
        }      
    }
    else
        mju_error("Unknown or missing mupooltype.");


    return firing_rate;

}

double fireNow(const mjModel* m, mjData* d, mjControl& c, double firing_rate, int actID)
{
    double fire;
    int nact = m->nuserdata / m->nu;
    double time_since_fired = d->userdata[actID * nact + 2];
    double ap_real_dur = d->userdata[actID * nact + 3];
    double last_firing_rate = d->userdata[actID * nact + 4];

    if (d->ctrl[actID] > 0.0 && ap_real_dur < c.ap_dur)
    { 
        fire = 1.0;
    }   
    else
    {
        if (firing_rate > 0.0)
        {
            double inter_impulse = 1.0 / firing_rate;

            if (inter_impulse <= time_since_fired)
            {
                fire = 1.0;
                if (last_firing_rate < 0.0001)
                    time_since_fired = 0.0;
                else
                {
                    time_since_fired = m->opt.timestep - (1 / last_firing_rate - time_since_fired + m->opt.timestep) /
                        (1 - (inter_impulse - 1 / last_firing_rate) / m->opt.timestep);
                }
            }
            else
            {
                fire = 0.0; 
            }

        }
        else
        {
            fire = 0.0;
        }
    }

    if (fire > 0.0)
        ap_real_dur += m->opt.timestep;
    else
        ap_real_dur = 0.0;

    d->userdata[actID * nact + 2] = time_since_fired + m->opt.timestep;
    d->userdata[actID * nact + 3] = ap_real_dur;
    d->userdata[actID * nact + 4] = firing_rate;

    return fire;
}

void muscleControlTargetCompTorq(const mjModel* m, mjData* d, mjData* dpred, const bool use_predicted_data, mjControl& c)
{
    /*Computes input control signal for muscles based on the error between target and grip site
    positions using the computed torque approach.
    */

    mjtNum* PDmagn = new mjtNum[m->nv];
    mjtNum* computed_torque = new mjtNum[m->nv];

    if (use_predicted_data)
    {   
        computePDmagn(m, dpred, c, PDmagn);
        mj_mulM(m, dpred, computed_torque, PDmagn);
        mju_addTo(computed_torque, dpred->qfrc_bias, m->nv);
    }
    else
    {
        computePDmagn(m, d, c, PDmagn);
        mj_mulM(m, d, computed_torque, PDmagn);
        mju_addTo(computed_torque, d->qfrc_bias, m->nv);
    }

    double act_torque;
    double max_torque;
    int best_actuator;

    mju_zero(d->ctrl, m->nu);

    for (int qID = 0; qID < m->nv; qID++)
    {
        max_torque = 0.0;
        for (int actID = 0; actID < m->nu; actID++)
        {
            if (use_predicted_data)
                act_torque = d->actuator_moment[actID * m->nv + qID] * muscleGainFLV(m, dpred, actID);
            else
                act_torque = d->actuator_moment[actID * m->nv + qID] * muscleGainFLV(m, d, actID);

            if (act_torque * computed_torque[qID] > 0.0 && abs(act_torque) > abs(max_torque)) // highest torque in right direction
            {
                best_actuator = actID;
                max_torque = act_torque;
            }
        }

        if (abs(max_torque) > 0)
        {
            d->ctrl[best_actuator] = abs(computed_torque[qID] / max_torque);
            if (d->ctrl[best_actuator] > 1.0)
                d->ctrl[best_actuator] = 1.0;
        }
    }
    delete[] PDmagn;
    delete[] computed_torque;
}

void muscleControlTargetCompTorqRate(const mjModel* m, mjData* d, mjData* dpred, const bool use_predicted_data, mjControl& c)
{
    /*Computes input control signal for muscles based on the error between target and grip site
    positions using the computed torque approach.
    */

    mjtNum* PDmagn = new mjtNum[m->nv];
    mjtNum* computed_torque = new mjtNum[m->nv];

    if (use_predicted_data)
    {
        computePDmagn(m, dpred, c, PDmagn);
        mj_mulM(m, dpred, computed_torque, PDmagn);
        mju_addTo(computed_torque, dpred->qfrc_bias, m->nv);
    }
    else
    {
        computePDmagn(m, d, c, PDmagn);
        mj_mulM(m, d, computed_torque, PDmagn);
        mju_addTo(computed_torque, d->qfrc_bias, m->nv);
    }

    int groupID;
    int antgrpID;
    double desired_force;
    double FLV_gain;
    double fr;

    for (int qID = 0; qID < m->nv; qID++)
    {
        groupID = (int)(computed_torque[qID] < 0) + qID * 2;
        antgrpID = (int)(computed_torque[qID] >= 0) + qID * 2;

        for (int actID = 0; actID < m->nu; actID++)
        {
            if (m->actuator_group[actID] == groupID)
            {
                if (use_predicted_data)
                    FLV_gain = muscleGainFLV(m, dpred, actID) / muscleGainConst(m, dpred, actID);
                else
                    FLV_gain = muscleGainFLV(m, d, actID) / muscleGainConst(m, d, actID);
                desired_force = computed_torque[qID] / d->actuator_moment[actID * m->nv + qID];
               

                fr = desiredForce2Rate(m, desired_force, FLV_gain, actID);
                d->ctrl[actID] = fireNow(m, d, c, fr, actID);
            }
            else if (m->actuator_group[actID] == antgrpID)
            {
                fr = 0.0;
                d->ctrl[actID] = fireNow(m, d, c, fr, actID);
            }
        }
    }
    delete[] PDmagn;
    delete[] computed_torque;
}