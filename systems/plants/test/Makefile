
eigenflags = "-I../../../thirdParty/eigen3"

urdf_kin_test : urdf_kin_test.cpp ../urdf.o ../RigidBodyManipulator.o ../link.o ../joint.o
	g++ -g $^ -o $@ $(eigenflags) -ltinyxml
