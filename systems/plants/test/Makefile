
EIGEN_CFLAGS = $(shell pkg-config --cflags eigen3)

all : urdf_kin_test

urdf_kin_test : urdf_kin_test.cpp ../urdf.o ../RigidBodyManipulator.o ../urdf_interface/link.o ../urdf_interface/joint.o
	g++ -O3 $^ -o $@ $(EIGEN_CFLAGS) -ltinyxml -I.. -lboost_filesystem-mt -lboost_system-mt  $(shell pkg-config --libs bot2-vis)

clean:	
	-rm -r urdf_kin_test
