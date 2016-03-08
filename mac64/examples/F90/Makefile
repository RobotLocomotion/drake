# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

CC     = gcc
CFLAGS = -DFNAME_LCASE_DECOR -I$(PATHDIR)/include -m64
F90    = gfortran
FFLAGS = -m64
SHARED_PATHLIB = -L$(PATHLIB) -lpath47
# we list the .a explicitly: the OSX linker will prefer the .dylib over the .a otherwise
STATIC_PATHLIB = $(PATHLIB)/libpath47.a

COBJS = Standalone_Path.o StandaloneF_Output.o

all: linmcp_shared linmcp_static

linmcp_shared: linmcp.f90 fout.f90 $(COBJS)
	$(F90) $(FFLAGS) -o $@ $^ $(SHARED_PATHLIB) -lm

linmcp_static: linmcp.f90 fout.f90 $(COBJS)
	$(F90) $(FFLAGS) -o $@ $^ $(STATIC_PATHLIB) -lm

%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

test:
	./linmcp_shared 6
	./linmcp_static 6
	@echo "-------------------------------------------------------------"
	@echo "---- demo-size test passed: trying larger instances next ----"
	@echo "-------------------------------------------------------------"
	./linmcp_shared 20
	./linmcp_static 20
