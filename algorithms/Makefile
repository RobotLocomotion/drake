
CC = gcc
CXX = g++

MATLABROOT = $(strip $(shell cat ../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    
MATLABBIN = $(MATLABROOT)/bin/$(strip $(shell cat ../.matlabcpu))

MEX_CPP_FILES = fastQPmex.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

SUBDIRS =

all: CXX += -O3
all: MEX += -O
all : $(MEXFILES)
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done

debug: CXX += -g
debug: MEX += -g
debug: $(MEXFILES)
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir debug || exit 2; \
	done

%.$(MEXEXT) : %.cpp
	$(MEX) $^ $(CFLAGS)

fastQPmex.$(MEXEXT) : fastQPmex.cpp fastQP.o
	$(MEX) $^ $(CFLAGS) $(shell pkg-config --cflags eigen3)

fastQP.o :fastQP.cpp fastQP.h
	$(CXX) -c $< $(shell pkg-config --cflags eigen3)

clean:
	-rm -f *.$(MEXEXT)
	-rm *.o
