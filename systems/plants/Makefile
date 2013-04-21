
eigenflags = -I../../thirdParty/eigen3

all : urdf.o joint.o link.o

urdf.o : urdf.cpp RigidBodyManipulator.h RigidBody.h
	g++ -O3 $< -c $(eigenflags) -I.  $(shell pkg-config --cflags bot2-vis)

joint.o : urdf_interface/joint.cpp 
	g++ -O3 $< -c -I.

link.o : urdf_interface/link.cpp 
	g++ -O3 $< -c -I.
