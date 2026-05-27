
MUJOCO_LOC = ../mujoco
ALGLIB_LOC = ../../alglib-cpp
MJ_SETTINGS = -O2 -I$(MUJOCO_LOC)/include -L$(MUJOCO_LOC)/bin -std=c++11 -Wl,-R$(MUJOCO_LOC)/bin
ALGLIB = -I$(ALGLIB_LOC)/src $(ALGLIB_LOC)/src/alglibinternal.cpp $(ALGLIB_LOC)/src/alglibmisc.cpp $(ALGLIB_LOC)/src/ap.cpp $(ALGLIB_LOC)/src/linalg.cpp 

all:
	g++ $(MJ_SETTINGS) -I../inc simulate_reaches.cpp -lmujoco210 -lGL -lglew $(MUJOCO_LOC)/bin/libglfw.so.3 $(ALGLIB) -o simulate_reaches 	
