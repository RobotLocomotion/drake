
CXX = g++

EIGEN_CFLAGS = $(shell pkg-config --cflags eigen3)
BULLET_CFLAGS = $(shell pkg-config --cflags bullet)
BULLET_LIBS = $(shell pkg-config --libs-only-L bullet) -lBulletCollision -lLinearMath

all: CXX += -O3
all : urdf_kin_test

debug: CXX += -g
debug: urdf_kin_test

urdf_kin_test : urdf_kin_test.cpp ../urdf.o ../RigidBodyManipulator.o ../urdf_interface/link.o ../urdf_interface/joint.o
	$(CXX) $^ -o $@ $(EIGEN_CFLAGS) -ltinyxml -I.. -lboost_filesystem-mt -lboost_system-mt  $(shell pkg-config --libs bot2-vis) $(BULLET_CFLAGS) $(BULLET_LIBS)

clean:	
	-rm -r urdf_kin_test
