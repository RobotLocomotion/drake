# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

CXX = cl
CXXFLAGS = -DFNAME_LCASE_NODECOR -I$(PATHDIR)/include

all: obstacle_c_shared.exe obstacle_c_static.exe

obstacle_c_shared.exe: obstacle_c.cpp cWrapper_Path.cpp
	$(CXX) $(CXXFLAGS) -DUSE_OUTPUT_INTERFACE -nologo -Fe$@ $^ $(PATHLIB)/path47.lib

obstacle_c_static.exe: obstacle_c.cpp cWrapper_Path.cpp
	$(CXX) -D_PTH_STATIC_LIB_ $(CXXFLAGS) -nologo -Fe$@ $^ $(PATHLIB)/path47_static.lib

test:
	./obstacle_c_shared.exe 6
	./obstacle_c_static.exe 6
	@echo "-------------------------------------------------------------"
	@echo "---- demo-size test passed: trying larger instances next ----"
	@echo "-------------------------------------------------------------"
	./obstacle_c_shared.exe 20
	./obstacle_c_static.exe 20
