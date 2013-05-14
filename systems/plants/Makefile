
EIGEN_CFLAGS = $(shell pkg-config --cflags eigen3)
BOTVIS_CFLAGS = $(shell pkg-config --cflags bot2-vis)

CFLAGS = 
CFLAGS = -g   # uncomment this to debug.  todo: figure out the best way to pass this up the directory chain?

SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)

MATLABROOT = $(strip $(shell cat ../../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPPFILES = deleteModelmex.cpp \
	HandCmex.cpp \
	doKinematicsmex.cpp \
	forwardKinmex.cpp \
	deleteModelpmex.cpp \
	HandCpmex.cpp \
	forwardKinpmex.cpp \
	forwardKinVelpmex.cpp 

MEXFILES = $(MEX_CPPFILES:%.cpp=%.$(MEXEXT)) 

all : $(MEXFILES) #urdf.o 
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done

RigidBodyManipulator.o : RigidBodyManipulator.cpp RigidBodyManipulator.h RigidBody.h
	g++ -O3 $< -c $(EIGEN_CFLAGS) $(CFLAGS)

%.$(MEXEXT) : %.cpp RigidBodyManipulator.o
	$(MEX) -O $^ $(EIGEN_CFLAGS) $(CFLAGS)

urdf.o : urdf.cpp RigidBodyManipulator.o
	g++ -O3 $< -c $(EIGEN_CFLAGS) $(BOTVIS_CFLAGS) $(CFLAGS)


clean: 
	-rm -f $(MEXFILES) RigidBodyManipulator.o urdf.o
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done
