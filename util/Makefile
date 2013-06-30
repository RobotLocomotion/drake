
CC = gcc
CXX = g++

MATLABROOT = $(strip $(shell cat ../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    
MATLABBIN = $(MATLABROOT)/bin/$(strip $(shell cat ../.matlabcpu))

MEX_CPP_FILES = realtime.cpp deleteMexPointer.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

EXES = debugMex

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

deleteMexPointer.$(MEXEXT) : deleteMexPointer.cpp drakeUtil.o

$(MEXFILES): %.$(MEXEXT) : %.cpp
	$(MEX) $^ $(CFLAGS)

$(BINDIR)/debugMex: debugMex.cpp
	$(CXX) -o $(BINDIR)/debugMex debugMex.cpp -I$(MATLABROOT)/extern/include -L$(MATLABBIN) -lmat -lmx -Wl,-rpath,$(MATLABBIN)

clean:
	-rm -f *.$(MEXEXT)
	-rm -f $(BINEXES)
