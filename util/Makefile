
CC = gcc
CXX = g++

MATLABROOT = $(strip $(shell cat ../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

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
	$(CC) -o debugMex debugMex.cpp -I$(MATLABROOT)/extern/include -L$(MATLABROOT)/bin/glnxa64 -lmat -lmx -Wl,-rpath,$(MATLABROOT)/bin/glnxa64

clean:
	-rm -f *.$(MEXEXT)
	-rm debugMex
