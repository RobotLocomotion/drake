
CC = g++

# urdf interface and tinyxml
CFLAGS = -I.. -I../tinyxml
LDFLAGS = ../RigidBodyManipulator.o ../urdf.o ../urdf_interface/link.o ../urdf_interface/joint.o ../tinyxml/tinyxml.o ../tinyxml/tinyxmlparser.o ../tinyxml/tinyxmlerror.o 

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
CFLAGS += $(shell pkg-config --cflags bullet)
LDFLAGS += $(shell pkg-config --libs-only-L bullet) -lBulletCollision -lLinearMath

all: CC += -O3
all: drake_viewer

debug: CC += -g
debug: drake_viewer

drake_viewer : main.cpp drake_urdf_renderer.o drake_urdf_renderer.h 
	$(CC) $^ -o $@ $(CFLAGS) $(LDFLAGS) -lboost_filesystem-mt -lboost_system-mt 

%.o : %.cpp %.h
	$(CC) -c $< $(CFLAGS)

clean: 
	-rm -rf *.o drake_viewer*
