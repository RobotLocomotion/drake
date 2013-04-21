
CC = g++
CFLAGS = $(shell pkg-config --cflags bot2-vis bot2-lcmgl-renderer) -O3
LDFLAGS = $(shell pkg-config --libs bot2-vis bot2-lcmgl-renderer) -ltinyxml
# note: for DRC folks, these will work if PKG_CONFIG_PATH points to 
#    drc/software/build/lib/pkgconfig
eigenflags = -I../../../thirdParty/eigen3

drake_viewer : main.cpp drake_urdf_renderer.o drake_urdf_renderer.h ../urdf.o ../RigidBodyManipulator.o ../link.o ../joint.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

%.o : %.cpp %.h
	$(CC) -c $(CFLAGS) $(eigenflags) -I. $<

../%.o : 
	$(MAKE) -C .. $(@:../%.o=%.o)
