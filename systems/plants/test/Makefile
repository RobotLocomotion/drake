
CXX = g++

# urdf interface and tinyxml
CFLAGS = -I.. -I../tinyxml
LDFLAGS = ../urdf_interface/link.o ../urdf_interface/joint.o ../tinyxml/tinyxml.o ../tinyxml/tinyxmlparser.o ../tinyxml/tinyxmlerror.o 

# eigen3
CFLAGS += $(shell pkg-config --cflags eigen3)
LDFLAGS += $(shell pkg-config --libs)

# boost
LDFLAGS += -L/opt/local/lib -lboost_filesystem-mt -lboost_system-mt

# bot-vis (required for urdf.cpp)
#LDFLAGS += $(shell pkg-config --libs bot2-vis)

# add bullet
#CFLAGS += $(shell pkg-config --cflags bullet)
#LDFLAGS += $(shell pkg-config --libs-only-L bullet) -lBulletCollision -lLinearMath

all: CXX += -O3
all : urdf_kin_test

debug: CXX += -g
debug: urdf_kin_test

urdf_kin_test : urdf_kin_test.cpp ../urdf.o ../RigidBodyManipulator.o 
	$(CXX) $^ -o $@ $(CFLAGS) $(LDFLAGS)

clean:	
	-rm -r urdf_kin_test
