
MATLABROOT = $(strip $(shell cat ../../../../.matlabroot))
MEX = $(MATLABROOT)/bin/mex
MEXEXT = $(shell $(MATLABROOT)/bin/mexext)    

MEX_CPP_FILES = constructModelmex.cpp
MEXFILES = $(MEX_CPP_FILES:%.cpp=%.$(MEXEXT))

CFLAGS = $(shell pkg-config --cflags eigen3)
LDFLAGS = 

# add bullet
CFLAGS += $(shell pkg-config --cflags bullet)
LDFLAGS += $(shell pkg-config --libs-only-L bullet) -lBulletCollision -lLinearMath

all : MEX += -O
all : $(MEXFILES)

debug: MEX += -g
debug: $(MEXFILES)

$(MEXFILES): %.$(MEXEXT) : %.cpp ../../RigidBodyManipulator.o 
	$(MEX) $^ $(CFLAGS) $(LDFLAGS)

clean:
	-rm -f *.$(MEXEXT)
