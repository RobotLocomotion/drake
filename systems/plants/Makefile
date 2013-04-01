
eigenflags = -I../../thirdParty/eigen3

all : urdf.o joint.o link.o

urdf.o : urdf.cpp
	g++ -g $< -c $(eigenflags) -I.

joint.o : urdf_interface/joint.cpp 
	g++ -O3 $< -c -I.

link.o : urdf_interface/link.cpp 
	g++ -O3 $< -c -I.
