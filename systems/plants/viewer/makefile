
CC = g++
CFLAGS = $(shell pkg-config --cflags bot2-vis bot2-lcmgl-renderer) -O3 -lstdc++
LDFLAGS = $(shell pkg-config --libs bot2-vis bot2-lcmgl-renderer)
# note: for DRC folks, these will work if PKG_CONFIG_PATH points to 
#    drc/software/build/lib/pkgconfig
eigenflags = -I../../../thirdParty/eigen3

viewer : main.cpp drake_urdf_renderer.o drake_urdf_renderer.h
	$(CC) $(CFLAGS) main.cpp -o $@ $(LDFLAGS)

%.o : %.cpp %.h
	$(CC) -c $(CFLAGS) $(eigenflags) -I. $<
