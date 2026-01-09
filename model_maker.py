from dm_control import mjcf
import numpy as np

import matplotlib.pyplot as plt


def equivalentPoolParams(n,min_threshold_frac,max_threshold_frac):

    n_ref = 600
    Rf_ref = 100     
    min_threshold_frac_ref = 0.001
    max_threshold_frac_ref = 0.9

    Rr = 2.4
    c1 = 0.075
    c2 = 1.8
    d1 = 0.0181
    d2 = 1.5

    if n>1:
        mu_ind = np.arange(n) / (n-1)
    else:
        mu_ind = 0

    # fmax scaling
    Rf = Rf_ref*(1-pow(Rf_ref,(-1/n))) / (pow(Rf_ref,(1/n))-1)

    fmax_scale = pow(Rf, mu_ind)
    fmax_scale = fmax_scale / np.sum(fmax_scale)

    if n>1:
        act_scale = pow(fmax_scale / fmax_scale[-1],-np.log(3)/np.log(Rf))
    else:
        act_scale = 1

    # recruitment thresholds
    Rc = (max_threshold_frac - c1) / min_threshold_frac
    mu_samp = np.arange(n+1) / (n)
    min_threshold_full = min_threshold_frac * pow(Rc, pow(mu_samp, c2)) + c1 * mu_samp
    min_threshold = min_threshold_full[0:n]
    ref_drive_full = d1 * pow(Rr, pow(mu_samp, d2))
    rate_band_full = min_threshold_full / (1-np.exp(min_threshold_full / ref_drive_full * (1-1/0.9)))
    rate_band = rate_band_full[1:n+1]

    Rd = (1-1/0.9)*min_threshold / np.log(1-min_threshold/rate_band)

    # rate coding reference drive
    if n == 1:
        d1 = Rd
    if n == 2:
        d1 = Rd[0]
        Rr = Rd[1]/d1
    if n>2:
        d1 = Rd[0]
        Rr = Rd[n-1]/d1
        mid_ind = int(np.ceil(n/2)-1)
        d2 = (np.log(np.log(Rd[mid_ind])-np.log(d1))-np.log(np.log(Rr)))/np.log(mu_ind[mid_ind])
    ref_drive = d1 * pow(Rr, pow(mu_ind, d2))

    return fmax_scale, min_threshold, ref_drive, act_scale

def makeMultiMU(mjcf_model,n,rmax,mupooltype):

    timestep = mjcf_model.option.timestep
    
    # find all actuators to divide into MUs
    actuators = mjcf_model.find_all('actuator')


    # set parameters for first MU
    min_rate = 0.0 
    max_rate_1 = rmax 
    min_threshold_frac = 0.001
    max_threshold_frac = 0.9

    if mupooltype == 'exp-lin':
        mjcf_model.find('numeric','mupool').data[0] = 0
        fmax_scale = equivalentPoolParams(n, min_threshold_frac, max_threshold_frac)
    elif mupooltype == 'mixed-log':
        mjcf_model.find('numeric','mupool').data[0] = 1
        fmax_scale, min_threshold_scale, ref_drive_scale, act_scale  = equivalentPoolParams(n, min_threshold_frac, max_threshold_frac)
    else:
        print("Unknown mupooltype.")

    # activation parameters
    gen_dyn =  [0.0140,  0.7, 0.0310,  0.8, 0.0185,  0.6]

    for i in range(len(actuators)):

        # identify tendon & muscle group
        temp_tendon = actuators[i].tendon
        muscle_group = actuators[i].group

        # total muscle strength
        Fmax = actuators[i].gainprm[0]
        first_recr = min_threshold_frac * abs(Fmax)
        last_recr = max_threshold_frac * abs(Fmax)

        # parameters for pools & first MU
        if mupooltype == 'exp-lin':
            Rd = last_recr / first_recr
            recruitment_bw = abs(Fmax) - last_recr
            user_params = [1, min_rate, max_rate_1, first_recr, first_recr+recruitment_bw, 1]
            # user_params above: [l0 (runtime), min_rate, max_rate, min_thr, max_thr, amax (runtime)]
        elif mupooltype == 'mixed-log':
            if n == 1:
                ref_drive = ref_drive_scale*abs(Fmax)/rmax
                min_threshold = min_threshold_scale*abs(Fmax)
            else:
                ref_drive = ref_drive_scale[0]*abs(Fmax)/rmax
                min_threshold = min_threshold_scale[0]*abs(Fmax)

            user_params = [1, min_rate, max_rate_1, first_recr, ref_drive, 1]
            # user_params above: [l0 (runtime), n/a, max_rate, min_thr, ref_drive, amax (runtime)]
        else:
            print("Unknown mupooltype.")
        
        # set parameters of template actuator to be MU 1
        if n>1:
            actuators[i].gainprm[0] = fmax_scale[0] * Fmax
            actuators[i].dynprm = [act_scale[0]*gen_dyn[0], gen_dyn[1], act_scale[0]*gen_dyn[2], gen_dyn[3], act_scale[0]*gen_dyn[4], gen_dyn[5]]
        else:
            actuators[i].dynprm = gen_dyn
        actuators[i].user = user_params
        
        
        if n>1:
            for j in range(1,n):
                # name and add new MU to model
                temp_name = actuators[i].name + str(j)
                mjcf_model.actuator.add('general',name=temp_name, group=muscle_group, tendon=temp_tendon)

                if mupooltype == 'exp-lin':
                    min_threshold = first_recr * pow(Rd, mu_ind[j])
                    user_params = [1, min_rate, max_rate_1, min_threshold, min_threshold+recruitment_bw, 1]
                elif mupooltype == 'mixed-log':
                    ref_drive = ref_drive_scale[j]*abs(Fmax)/rmax
                    min_threshold = min_threshold_scale[j]*abs(Fmax)
                    user_params = [1, min_rate, max_rate_1, min_threshold, ref_drive, 1]
                else:
                    print("Unknown mupooltype.")

                # set parameter values for new MU
                mjcf_model.find('actuator',temp_name).user = user_params
                mjcf_model.find('actuator',temp_name).gainprm = actuators[i].gainprm
                mjcf_model.find('actuator',temp_name).gainprm[0] = fmax_scale[j] * Fmax
                mjcf_model.find('actuator',temp_name).biasprm = actuators[i].biasprm
                mjcf_model.find('actuator',temp_name).dynprm = [act_scale[j]*gen_dyn[0], gen_dyn[1], act_scale[j]*gen_dyn[2], gen_dyn[3], act_scale[j]*gen_dyn[4], gen_dyn[5]]

        mjcf_model.size.nuserdata = 5*n*len(actuators)
        mjcf_model.size.nuser_actuator = 6


def setPassiveStiffness(mjcf_model, sp, rp, lp):
    actuators = mjcf_model.find_all('actuator')

    for i in range(len(actuators)):
        actuators[i].biasprm = [actuators[i].gainprm[0]*sp, rp, lp]


template_filename = "human_arm_3dof_A_1MU_template.xml"

ns = [20, 50];
rmaxs = [30, 75];

for i in ns:
    for j in rmaxs:
        output_filename = "C:/Users/tmurtola/source/mujoco200_WT/models/test_arm_3dof_A_" + str(i) + "MU_" + str(j) + "Hz_fbr_mixed_log.xml"

        # Parse from file
        with open(template_filename) as f:
            mjcf_model = mjcf.from_file(f)

        makeMultiMU(mjcf_model,i,j,'mixed-log')
        setPassiveStiffness(mjcf_model,0.05,5,1.1)

        XML = mjcf_model.to_xml_string()

        with open(output_filename, "w") as f:
            f.write(XML)

#print(XML)

