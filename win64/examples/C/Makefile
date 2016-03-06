# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

CC = icl
CFLAGS = -DFNAME_LCASE_NODECOR -I$(PATHDIR)/include

all: obstacle_shared.exe obstacle_static.exe

obstacle_shared.exe: obstacle.c Standalone_Path.c
	$(CC) $(CFLAGS) -DUSE_OUTPUT_INTERFACE -nologo -Fe$@ $^ $(PATHLIB)/path47.lib

obstacle_static.exe: obstacle.c Standalone_Path.c
	$(CC) -D_PTH_STATIC_LIB_ $(CFLAGS) -nologo -Fe$@ $^ $(PATHLIB)/path47_static.lib

test:
	./obstacle_shared.exe 6
	./obstacle_static.exe 6
	@echo "-------------------------------------------------------------"
	@echo "---- demo-size test passed: trying larger instances next ----"
	@echo "-------------------------------------------------------------"
	./obstacle_shared.exe 20
	./obstacle_static.exe 20
