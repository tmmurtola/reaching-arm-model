*********
reaching-arm-model
*********

This code package contains a simple MuJoCo arm model and control code that executes a series of reaching movements towards targets in the workspace.

Preliminaries
============
Download and install `MuJoCo <http://www.mujoco.org/index.html/>`_ with active license and the `ALGLIB library <https://www.alglib.net/>`_. The code has been developed with MuJoCo 2.0 for Windows and ALGLIB 3.16.0. Compatibility with other versions has not been tested.

The contents of this package:

* simulate_reaches.cpp - main C++ file
* reachingToolbox.h        - header file containing auxiliary and callback functions
* mjcontrol.h          - header file defining control variable structure
* human_arm_3dof.xml   - xml model file

To compile
=========
The main file simulate_reaches.cpp needs to be compiled with the alglib files alglibinternal.cpp, alglibmisc.cpp, ap.cpp, linalg.cpp, and the mujoco libraries glfw3.lib and  mujoco200.lib. E.g.::

  cl /O2 /MT /EHsc /arch:AVX /I../inc /Fe"simulate_reaches.exe" simulate_reaches.cpp glfw3.lib  mujoco200.lib alglibinternal.cpp alglibmisc.cpp ap.cpp linalg.cpp


To run
======
Call simulate_reaches with the chosen model file. E.g.::

  simulate_reaches human_arm_3dof.xml


To modify
========
Muscle force generation and activation dynamics functions can be varied by commenting/uncommeting appropriate callbacks on lines 179-182 in simulate_reaches.cpp. 

muscleGainFLV enables force-length-velocity properties for the muscles while muscleGainConst disables the FLV properties. 

muscleActivation1stOrder and muscleActivation3rdOrder implement first and third order activation dynamics, respectively. To use instantaneous activation, change default dyntype to "none" in the xml file with either mjcb_act_dyn, or none, specified in simulate_reaches.cpp. The parameters of the activation dynamics are defined in the xml model. Remember to alter them to match the chosen activation model.

Control parameters, including PD gains and prediction time (assumed delay) can be changed in the control structure definition file mjcontrol.h.

