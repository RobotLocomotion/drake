
MATLABROOT = $(strip $(shell cat ../../../../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPP_FILES = constructModelpmex.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

EIGEN_CFLAGS = $(shell pkg-config --cflags eigen3)

all : $(MEXFILES)

debug: MEX += -g
debug: $(MEXFILES)

$(MEXFILES): %.$(MEXEXT) : %.cpp 
	$(MEX) -O $^ $(EIGEN_CFLAGS) 

clean:
	-rm -f *.$(MEXEXT)
