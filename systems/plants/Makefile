
CC = gcc
CXX = g++

CFLAGS = -I. 
LDFLAGS = 

# eigen3
CFLAGS += $(shell pkg-config --cflags eigen3)

# bullet
#CFLAGS += $(shell pkg-config --cflags bullet)
#LDFLAGS += $(shell pkg-config --libs-only-L bullet) -lBulletCollision -lLinearMath

# botvis (not required for mex files)
BOTVIS_CFLAGS = 
#BOTVIS_CFLAGS = $(shell pkg-config --cflags bot2-vis)


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

all: CXX += -O3
all: CC += -O3
all: MEX += -O
all : $(MEXFILES) urdf.o 
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done

debug: CXX += -g
debug: CC += -g
debug: MEX += -g

debug: $(MEXFILES) urdf.o 
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir debug || exit 2; \
	done

RigidBodyManipulator.o : RigidBodyManipulator.cpp RigidBodyManipulator.h RigidBody.h
	$(CXX) $< -c $(CFLAGS)

%.$(MEXEXT) : %.cpp RigidBodyManipulator.o
	$(MEX) $^ $(CFLAGS) $(LDFLAGS)

urdf.o : urdf.cpp RigidBodyManipulator.o
	$(CXX) $< -c -Itinyxml $(CFLAGS) $(BOTVIS_CFLAGS) 

clean: 
	-rm -f $(MEXFILES) RigidBodyManipulator.o urdf.o
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done
