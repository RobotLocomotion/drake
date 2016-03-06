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

all: ijvalDriver_shared ijvalDriver_static xpDriver_shared xpDriver_static

ijvalDriver_shared: ijvalDriver.c simpleQP.c
	$(CC) $(CFLAGS) -o $@ $^ $(SHARED_PATHLIB) $(XLDIRS) $(XLIBS) -lm

ijvalDriver_static: ijvalDriver.c simpleQP.c
	$(CC) $(CFLAGS) -o $@ $^ $(STATIC_PATHLIB) $(XLDIRS) $(XLIBS) -lm

xpDriver_shared: xpDriver.c sparseQP.c
	$(CC) $(CFLAGS) -o $@ $^ $(SHARED_PATHLIB) $(XLDIRS) $(XLIBS) -lm

xpDriver_static: xpDriver.c sparseQP.c
	$(CC) $(CFLAGS) -o $@ $^ $(STATIC_PATHLIB) $(XLDIRS) $(XLIBS) -lm

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
