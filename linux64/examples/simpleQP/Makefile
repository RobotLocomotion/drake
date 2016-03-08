# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

# additional libs - we expect them to be required only for static builds
XLIBS = -lgfortran

CC    = gcc
CFLAGS = -I$(PATHDIR)/include -m64 -Wall

all: ijvalDriver_shared ijvalDriver_static xpDriver_shared xpDriver_static

ijvalDriver_shared: ijvalDriver.c simpleQP.c
	$(CC) $(CFLAGS) -o $@ $^         -L$(PATHLIB) -lpath47 -lm

ijvalDriver_static: ijvalDriver.c simpleQP.c
	$(CC) $(CFLAGS) -o $@ $^ -static -L$(PATHLIB) -lpath47 $(XLIBS) -lm

xpDriver_shared: xpDriver.c sparseQP.c
	$(CC) $(CFLAGS) -o $@ $^         -L$(PATHLIB) -lpath47 -lm

xpDriver_static: xpDriver.c sparseQP.c
	$(CC) $(CFLAGS) -o $@ $^ -static -L$(PATHLIB) -lpath47 $(XLIBS) -lm

test:
	@echo "-------------------------------------------------------------"
	@echo "---------- file da.dat should have obj of 21.0771 -----------"
	@echo "-------------------------------------------------------------"
	./ijvalDriver_shared sampleData/da.dat
	./ijvalDriver_static sampleData/da.dat
	@echo "-------------------------------------------------------------"
	@echo "------------- file da2.dat should be infeasible -------------"
	@echo "-------------------------------------------------------------"
	./ijvalDriver_shared sampleData/da2.dat
	./ijvalDriver_static sampleData/da2.dat
	@echo "-------------------------------------------------------------"
	@echo "-------------------------------------------------------------"
	( cd sampleData ; ../xpDriver_static tinyAscii.info )
