# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

CC     = gcc
CFLAGS = -I$(PATHDIR)/include -m64
SHARED_PATHLIB = -L$(PATHLIB) -lpath47
# we list the .a explicitly: the OSX linker will prefer the .dylib over the .a otherwise
STATIC_PATHLIB = $(PATHLIB)/libpath47.a

# extras needed for some gfortran installations, e.g. MCF laptop
XLDIRS = -L/usr/local/lib/x86_64
XLIBS  = -lgfortran

all: obstacle_shared obstacle_static

obstacle_shared: obstacle.c Standalone_Path.c
	$(CC) $(CFLAGS) -o $@ $^ $(SHARED_PATHLIB) $(XLDIRS) $(XLIBS) -lm

obstacle_static: obstacle.c Standalone_Path.c
	$(CC) $(CFLAGS) -o $@ $^ $(STATIC_PATHLIB) $(XLDIRS) $(XLIBS) -lm

# On Ferris laptop need
# gcc -I../../include -m64 -o obstacle obstacle.c Standalone_Path.c -L../../lib -L/usr/local/lib/x86_64 -lpath47 -lgfortran -lm

test:
	./obstacle_shared 6
	./obstacle_static 6
	@echo "-------------------------------------------------------------"
	@echo "---- demo-size test passed: trying larger instances next ----"
	@echo "-------------------------------------------------------------"
	./obstacle_shared 20
	./obstacle_static 20
