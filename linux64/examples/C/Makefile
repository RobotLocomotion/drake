# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

# additional libs - we expect them to be required only for static builds
XLIBS = -lgfortran

CC    = gcc
CFLAGS = -I$(PATHDIR)/include -m64

all: obstacle_shared obstacle_static

obstacle_shared: obstacle.c Standalone_Path.c
	$(CC) $(CFLAGS) -o $@ $^         -L$(PATHLIB) -lpath47 -lm

obstacle_static: obstacle.c Standalone_Path.c
	$(CC) $(CFLAGS) -o $@ $^ -static -L$(PATHLIB) -lpath47 $(XLIBS) -lm

test:
	./obstacle_shared 6
	./obstacle_static 6
	@echo "-------------------------------------------------------------"
	@echo "---- demo-size test passed: trying larger instances next ----"
	@echo "-------------------------------------------------------------"
	./obstacle_shared 20
	./obstacle_static 20
