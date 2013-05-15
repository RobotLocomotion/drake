
CC = g++
CFLAGS = $(shell pkg-config --cflags bot2-vis bot2-lcmgl-renderer) -I../../../include/
LDFLAGS = $(shell pkg-config --libs bot2-vis bot2-lcmgl-renderer) -ltinyxml ../../../drake.a
# note: for DRC folks, these will work if PKG_CONFIG_PATH points to 
#    drc/software/build/lib/pkgconfig
eigenflags = $(shell pkg-config --cflags eigen3)
BULLET_CFLAGS = $(shell pkg-config --cflags bullet)
BULLET_LIBS = $(shell pkg-config --libs-only-L bullet) -lBulletCollision -lLinearMath

all: CC += -O3
all: drake_viewer

debug: CC += -g
debug: drake_viewer

drake_viewer : main.cpp drake_urdf_renderer.o drake_urdf_renderer.h ../urdf.o ../RigidBodyManipulator.o ../urdf_interface/link.o ../urdf_interface/joint.o
	$(CC) $(CFLAGS) $(BULLET_CFLAGS) $(BULLET_LIBS) $^ -o $@ $(LDFLAGS) -lboost_filesystem-mt -lboost_system-mt 

%.o : %.cpp %.h
	$(CC) -c $(CFLAGS) $(BULLET_CFLAGS) $(eigenflags) -I.. $<

clean: 
	-rm -f *.o drake_viewer
