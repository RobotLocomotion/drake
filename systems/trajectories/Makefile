
CC = gcc
CXX = g++

CFLAGS = -I. 
LDFLAGS = 

# eigen3
CFLAGS += $(shell pkg-config --cflags eigen3)

# mex
MATLABROOT = $(strip $(shell cat ../../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPPFILES = ExpPlusPPTrajectoryEvalmex.cpp PPTmex.cpp

MEXFILES = $(MEX_CPPFILES:%.cpp=%.$(MEXEXT)) 

all: CXX += -O3
all: CC += -O3
all: MEX += -O
all : $(MEXFILES)

debug: CXX += -g
debug: CC += -g
debug: MEX += -g

debug: $(MEXFILES)

%.$(MEXEXT) : %.cpp
	$(MEX) $^ $(CFLAGS) $(LDFLAGS)

clean: 
	-rm -f $(MEXFILES)
