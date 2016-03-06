# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

CC     = icl
CFLAGS = -DFNAME_LCASE_NODECOR -I$(PATHDIR)/include
F90    = ifort
FFLAGS =

COBJS = Standalone_Path.obj StandaloneF_Output.obj
COBJS2 = Standalone_Path_static.obj StandaloneF_Output_static.obj
HFILES = Standalone_Path.h

all: linmcp_shared.exe linmcp_static.exe

linmcp_shared.exe: linmcp.f90 fout.f90 $(COBJS)
	$(F90) $(FFLAGS) -Fe$@ $^ $(PATHDIR)/lib/path47.lib

linmcp_static.exe: linmcp.f90 fout.f90 $(COBJS2)
	$(F90) $(FFLAGS) -Fe$@ $^ $(PATHDIR)/lib/path47_static.lib

$(COBJS) : %.obj: %.c $(HFILES)
	$(CC) -c $(CFLAGS) $<

$(COBJS2) : %.obj: %.c $(HFILES)
	$(CC) -c $(CFLAGS) -D_PTH_STATIC_LIB_ $<

%_static.c: %.c
	cp $< $@

test:
	./linmcp_shared.exe
	./linmcp_static.exe
