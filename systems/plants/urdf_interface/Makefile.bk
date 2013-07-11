
CXX = g++
CFLAGS = 

all: CXX += -O3
all: joint.o link.o

debug: CXX += -g
debug: joint.o link.o

%.o : %.cpp 
	$(CXX) $< -c $(CFLAGS) -I..

clean : 
	-rm -f *.o