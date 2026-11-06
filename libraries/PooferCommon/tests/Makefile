# Host tests for PooferCommon's drive logic (poofer_drive.cpp). No Arduino needed.
#   make -C libraries/PooferCommon/tests test
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra

drive_sim: drive_sim.cpp ../src/poofer_drive.cpp ../src/poofer_drive.h
	$(CXX) $(CXXFLAGS) -o $@ drive_sim.cpp ../src/poofer_drive.cpp

test: drive_sim
	./drive_sim

clean:
	rm -f drive_sim

.PHONY: test clean
