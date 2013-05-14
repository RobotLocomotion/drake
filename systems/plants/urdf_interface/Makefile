
CFLAGS = $(shell pkg-config --cflags tinyxml)

all: joint.o link.o

%.o : %.cpp 
	g++ -O3 $< -c $(CFLAGS) -I..

clean : 
	-rm -f *.o