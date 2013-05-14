
CXX = g++
CFLAGS = $(shell pkg-config --cflags tinyxml)

all: joint.o link.o

debug: CXX += -g
debug: all 

%.o : %.cpp 
	$(CXX) -O3 $< -c $(CFLAGS) -I..

clean : 
	-rm -f *.o