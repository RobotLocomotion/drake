
MATLABROOT = $(strip $(shell cat ../../../../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPP_FILES = constructModelmex.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

EIGEN_CFLAGS = $(shell pkg-config --cflags eigen3)
CFLAGS = 

all : $(MEXFILES)

$(MEXFILES): %.$(MEXEXT) : %.cpp ../../RigidBodyManipulator.o 
	$(MEX) -O $^ $(EIGEN_CFLAGS) $(CFLAGS)

clean:
	-rm -f *.$(MEXEXT)
