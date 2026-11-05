# Host simulator for rig_control: make -C cold-test-rig/tools/sim
FW  := ../../firmware/rig_control
LIB := ../../../libraries/PooferCommon/src
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra

rig_sim: rig_sim.cpp $(FW)/rig_core.cpp $(FW)/rig_core.h $(LIB)/poofer_drive.cpp $(LIB)/poofer_drive.h
	$(CXX) $(CXXFLAGS) -I$(FW) -I$(LIB) -o $@ rig_sim.cpp $(FW)/rig_core.cpp $(LIB)/poofer_drive.cpp

clean:
	rm -f rig_sim

.PHONY: clean
