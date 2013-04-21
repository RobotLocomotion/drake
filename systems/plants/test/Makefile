
eigenflags = "-I../../../thirdParty/eigen3"

urdf_kin_test : urdf_kin_test.cpp ../urdf.o ../RigidBodyManipulator.o ../link.o ../joint.o
	g++ -O3 $^ -o $@ $(eigenflags) -ltinyxml -I.. -lboost_filesystem-mt -lboost_system-mt  $(shell pkg-config --libs bot2-vis)

../%.o : 
	$(MAKE) -C .. $(@:../%.o=%.o)

