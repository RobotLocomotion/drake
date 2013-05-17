
MATLABROOT = $(strip $(shell cat ../../../../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPP_FILES = constructModelmex.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

EIGEN_CFLAGS = $(shell pkg-config --cflags eigen3)
#BULLET_CFLAGS = $(shell pkg-config --cflags bullet)
#BULLET_LIBS = $(shell pkg-config --libs-only-L bullet) -lBulletCollision -lLinearMath
BULLET_CFLAGS =
BULLET_LIBS = 

all : MEX += -O
all : $(MEXFILES)

debug: MEX += -g
debug: $(MEXFILES)

$(MEXFILES): %.$(MEXEXT) : %.cpp ../../RigidBodyManipulator.o 
	$(MEX) $^ $(EIGEN_CFLAGS) $(BULLET_CFLAGS) $(BULLET_LIBS)

clean:
	-rm -f *.$(MEXEXT)
