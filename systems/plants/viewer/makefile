
CC = g++

# urdf interface and tinyxml
CFLAGS = -I.. -I../tinyxml
LDFLAGS = ../urdf_interface/link.o ../urdf_interface/joint.o ../tinyxml/tinyxml.o ../tinyxml/tinyxmlparser.o ../tinyxml/tinyxmlerror.o

# eigen3
CFLAGS += $(shell pkg-config --cflags eigen3)
LDFLAGS += $(shell pkg-config --libs)

# boost
LDFLAGS += -L/opt/local/lib -lboost_filesystem-mt -lboost_system-mt

# lcm and bot-vis 
CFLAGS += $(shell pkg-config --cflags lcm bot2-vis bot2-lcmgl-renderer)
LDFLAGS += $(shell pkg-config --libs lcm bot2-vis bot2-lcmgl-renderer)

# drake lcm types
CFLAGS += -I../../../include
LDFLAGS += ../../../drake.a

# add bullet
CFLAGS += $(shell pkg-config --cflags bullet) -DBULLET_COLLISION
LDFLAGS += $(shell pkg-config --libs-only-L bullet) -lBulletCollision -lLinearMath

BINDIR = ../../../bin

all: CC += -O3
all: $(BINDIR)/drake_viewer lcmgl_test

debug: CC += -g
debug: $(BINDIR)/drake_viewer lcmgl_test

$(BINDIR)/drake_viewer : main.cpp drake_urdf_renderer.o drake_urdf_renderer.h ../urdf.o ../RigidBodyManipulator.o
	$(CC) $^ -o $@ $(CFLAGS) $(LDFLAGS)

lcmgl_test : lcmgl_test.cpp
	$(CC) $^ -o $@ $(shell pkg-config --cflags --libs lcm bot2-lcmgl-client)

%.o : %.cpp %.h
	$(CC) -c $< $(CFLAGS)

clean: 
	-rm -rf *.o $(BINDIR)/drake_viewer*
