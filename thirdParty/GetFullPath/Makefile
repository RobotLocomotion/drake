
MATLABROOT = $(strip $(shell cat ../../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_C_FILES = GetFullPath.c
MEXFILES = $(MEX_C_FILES:%.c=%.$(MEXEXT))

CFLAGS = 

all : $(MEXFILES)

$(MEXFILES): %.$(MEXEXT) : %.c
	$(MEX) -O $^ $(CFLAGS)

clean:
	-rm -f *.$(MEXEXT)
