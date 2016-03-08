# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

CC = icl
CFLAGS = -DFNAME_LCASE_NODECOR -I$(PATHDIR)/include

all: ijvalDriver_shared.exe ijvalDriver_static.exe xpDriver_shared.exe xpDriver_static.exe

ijvalDriver_shared.exe: ijvalDriver.c simpleQP.c
	$(CC) $(CFLAGS) -DUSE_OUTPUT_INTERFACE -nologo -Fe$@ $^ $(PATHLIB)/path47.lib

ijvalDriver_static.exe: ijvalDriver.c simpleQP.c
	$(CC) -D_PTH_STATIC_LIB_ $(CFLAGS) -nologo -Fe$@ $^ $(PATHLIB)/path47_static.lib

xpDriver_shared.exe: xpDriver.c sparseQP.c
	$(CC) $(CFLAGS) -DUSE_OUTPUT_INTERFACE -nologo -Fe$@ $^ $(PATHLIB)/path47.lib

xpDriver_static.exe: xpDriver.c sparseQP.c
	$(CC) -D_PTH_STATIC_LIB_ $(CFLAGS) -nologo -Fe$@ $^ $(PATHLIB)/path47_static.lib

test:
	@echo "-------------------------------------------------------------"
	@echo "---------- file da.dat should have obj of 21.0771 -----------"
	@echo "-------------------------------------------------------------"
	./ijvalDriver_shared.exe sampleData/da.dat
	./ijvalDriver_static.exe sampleData/da.dat
	@echo "-------------------------------------------------------------"
	@echo "------------- file da2.dat should be infeasible -------------"
	@echo "-------------------------------------------------------------"
	./ijvalDriver_shared.exe sampleData/da2.dat
	./ijvalDriver_static.exe sampleData/da2.dat
	@echo "-------------------------------------------------------------"
	@echo "-------------------------------------------------------------"
	( cd sampleData ; ../xpDriver_static.exe tinyAscii.info )
