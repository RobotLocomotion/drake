
all: joint.o link.o

%.o : %.cpp 
	g++ -O3 $< -c -I..

clean : 
	-rm -f *.o