
MATLABROOT = $(strip $(shell cat ../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPP_FILES = realtime.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

CFLAGS = 

all : $(MEXFILES)

$(MEXFILES): %.$(MEXEXT) : %.cpp
	$(MEX) -O $^ $(CFLAGS)

clean:
	-rm -f *.$(MEXEXT)
