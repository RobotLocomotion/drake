
CC = gcc
CXX = g++

MATLABROOT = $(strip $(shell cat ../../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPP_FILES = debugMexTest.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

all : $(MEXFILES) 

debug: MEX += -g
debug: all 

$(MEXFILES): %.$(MEXEXT) : %.cpp
	$(MEX) -O $^ $(CFLAGS)

clean:
	-rm -f *.$(MEXEXT)
	-rm debugMex
