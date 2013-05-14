
CC = gcc
CXX = g++

EIGEN_CFLAGS = $(shell pkg-config --cflags eigen3)
BOTVIS_CFLAGS = $(shell pkg-config --cflags bot2-vis)
BULLET_CFLAGS = $(shell pkg-config --cflags bullet)
BULLET_LIBS = $(shell pkg-config --libs-only-L bullet) -lBulletCollision -lLinearMath

SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)

MATLABROOT = $(strip $(shell cat ../../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPPFILES = deleteModelmex.cpp \
	HandCmex.cpp \
	doKinematicsmex.cpp \
	forwardKinmex.cpp \
	deleteModelpmex.cpp \
	doKinematicspmex.cpp \
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

RigidBodyManipulator.o : RigidBodyManipulator.cpp RigidBodyManipulator.h RigidBody.h
	$(CXX) -O3 $< -c $(EIGEN_CFLAGS) $(BULLET_CFLAGS) 

%.$(MEXEXT) : %.cpp RigidBodyManipulator.o
	$(MEX) -O $^ $(EIGEN_CFLAGS) $(BULLET_CFLAGS) $(BULLET_LIBS)

urdf.o : urdf.cpp RigidBodyManipulator.o
	$(CXX) -O3 $< -c $(EIGEN_CFLAGS) $(BOTVIS_CFLAGS) 


clean: 
	-rm -f $(MEXFILES) RigidBodyManipulator.o urdf.o
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done
