
CC = gcc
CXX = g++

SUBDIRS = plants trajectories

MATLABROOT = $(strip $(shell cat ../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPP_FILES = DCSFunction.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

CFLAGS = 

all : $(MEXFILES)
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done

debug: CXX += -g
debug: CC += -g
debug: MEX += -g

debug: $(MEXFILES) 
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir debug || exit 2; \
	done

$(MEXFILES): %.$(MEXEXT) : %.cpp
	$(MEX) -O $^ $(CFLAGS)

clean:
	-rm -f *.$(MEXEXT)
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done
