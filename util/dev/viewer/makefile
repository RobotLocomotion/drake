
CC = gcc
CFLAGS = -O3 -lstdc++
FLAGS = $(shell pkg-config --cflags --libs bot2-vis bot2-lcmgl-renderer)
# note: for DRC folks, these will work if PKG_CONFIG_PATH points to 
#    drc/software/build/lib/pkgconfig

viewer : main.cpp
	$(CC) $(CFLAGS) main.cpp -o $@ $(FLAGS)
