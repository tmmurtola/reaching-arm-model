*********
reaching-arm-model (rate_coding)
*********

This code package contains a simple MuJoCo arm model and control code that executes a series of reaching movements towards targets in the workspace. Movement in the model is driven by rate-coded multi-motor unit muscles.

Preliminaries
============
Download and install `MuJoCo <http://www.mujoco.org/index.html/>`_ with active license and the `ALGLIB library <https://www.alglib.net/>`_. The code has been developed with MuJoCo 2.0 for Windows and ALGLIB 3.16.0. Compatibility with other versions has not been tested.

The contents of this package:

* simulate_reaches_rate.cpp  - source code for the model for generating full set of simulation data
* mjx_parallel_pop.cpp       - C++ source code that can be compiled to a mex-file for MATLAB
* model_maker                - Python script for generating MuJoCo models with multi-MU muscles
* human_arm_1MU_template.xml - xml model file that is used by model_maker to generate multi-MU models
* inc/                       - folder containing header files
* scripts/                   - folder containing MATLAB scripts for parameter optimisation, simulating workspace errors, and plotting results
* data/                      - optimised parameters, simulation data, and workspace errors used in publication


To compile
=========
The main file simulate_reaches_rate.cpp needs to be compiled with the alglib files alglibinternal.cpp, alglibmisc.cpp, ap.cpp, linalg.cpp, and the mujoco libraries glfw3.lib and  mujoco200.lib. E.g.::

  cl /O2 /MT /EHsc /arch:AVX /I../inc /Fe"simulate_reaches_rate.exe" simulate_reaches_rate.cpp glfw3.lib  mujoco200.lib alglibinternal.cpp alglibmisc.cpp ap.cpp linalg.cpp

To compile the mex file in MATLAB:

  mex COMPFLAGS="$COMPFLAGS /openmp" -output mjx_simulate_reaches_rate -DWIN32 -D_WIN32 -I..\inc\  mjx_parallel_pop.cpp alglibinternal.cpp alglibmisc.cpp ap.cpp linalg.cpp -lmujoco200.lib -lglfw3.lib


To run simulations
======
Full set of simulation data can be generated with the C++ version of the model. Call simulate_reaches with the chosen model, control parameter, target location (optional), and output (optional) files. E.g.::

  simulate_reaches_rate model_file.xml T[target_location_file.txt] C[control_parameter_file.txt] O[output_file_name.txt]

The mex implementation is a minimal version of the C++ model, which only outputs the homing-in error for the simulation, but which can parallelise simulations either by control parameters or by target. For examples on how to use the mex function, see scripts/optimise_control_params.mlx and scripts/simulate_workspace_errors.mlx.


To regenarete full set of results
========
1. Use model_maker to generate model files for the selected number of MUs and maximum firing rate.
2. Compile mex file in MATLAB if necessary.
3. Run optimise_control_params.mlx to obtain control parameters for the model.
4. Run simulate_workspace_errors.mlx to calculate homing-in errors for targets covering the entire workspace.
5. Compile simulate_reaches_rate.cpp if necessary.
6. Run simulate_reaches_rate to generate full time signals for reaching simulations with model.


