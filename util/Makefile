
CC = gcc
CXX = g++

MATLABROOT = $(strip $(shell cat ../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    
MATLABBIN = $(MATLABROOT)/bin/$(strip $(shell cat ../.matlabcpu))
MATLABFLAGS = -I$(MATLABROOT)/extern/include -L$(MATLABBIN) -lmat -lmx -Wl,-rpath,$(MATLABBIN) -DMX_COMPAT_32

MEX_CPP_FILES = realtime.cpp deleteMexPointer.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

# for mac:
LIBEXT = dylib
LIBFLAGS = -dynamiclib -flat_namespace
# for everyone else:
#LIBEXT = so
#LIBFLAGS = -shared

EXES = debugMex libDrakeDebugMex.$(LIBEXT)

BINDIR = ../bin
BINEXES = $(EXES:%=$(BINDIR)/%)


SUBDIRS = test

all: MEX += -O
all : drakeUtil.o $(MEXFILES) $(BINEXES)
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done

debug: MEX += -g
debug: drakeUtil.o $(MEXFILES) $(BINEXES)
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir debug || exit 2; \
	done

drakeUtil.o : drakeUtil.cpp drakeUtil.h
	$(MEX) $< -c $(CFLAGS) # -fPIC

$(BINDIR)/libDrakeDebugMex.$(LIBEXT) : libDrakeDebugMex.cpp
	$(CXX) $(LIBFLAGS) -o $@ $^ $(MATLABFLAGS)

deleteMexPointer.$(MEXEXT) : deleteMexPointer.cpp drakeUtil.o

$(MEXFILES): %.$(MEXEXT) : %.cpp
	$(MEX) $^ $(CFLAGS)

$(BINDIR)/debugMex: debugMex.cpp
	$(CXX) -o $(BINDIR)/debugMex debugMex.cpp $(MATLABFLAGS)
clean:
	-rm -f *.$(MEXEXT)
	-rm -f $(BINEXES)
