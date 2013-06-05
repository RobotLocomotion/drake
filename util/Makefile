
CC = gcc
CXX = g++

MATLABROOT = $(strip $(shell cat ../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    
MATLABBIN = $(MATLABROOT)/bin/$(strip $(shell cat ../.matlabcpu))

MEX_CPP_FILES = realtime.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

SUBDIRS = test

all : $(MEXFILES) debugMex
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done

debug: MEX += -g
debug: all 
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir debug || exit 2; \
	done

$(MEXFILES): %.$(MEXEXT) : %.cpp
	$(MEX) -O $^ $(CFLAGS)

debugMex: debugMex.cpp
	$(CXX) -o debugMex debugMex.cpp -I$(MATLABROOT)/extern/include -L$(MATLABBIN) -lmat -lmx -Wl,-rpath,$(MATLABBIN)

clean:
	-rm -f *.$(MEXEXT)
	-rm debugMex
