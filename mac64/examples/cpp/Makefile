# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

CXX    = g++
CXXFLAGS = -I$(PATHDIR)/include -m64
SHARED_PATHLIB = -L$(PATHLIB) -lpath47
# we list the .a explicitly: the OSX linker will prefer the .dylib over the .a otherwise
STATIC_PATHLIB = $(PATHLIB)/libpath47.a

# extras needed for some gfortran installations, e.g. MCF laptop
XLDIRS = -L/usr/local/lib/x86_64
XLIBS  = -lgfortran

all: obstacle_c_shared obstacle_c_static

obstacle_c_shared: obstacle_c.cpp cWrapper_Path.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(SHARED_PATHLIB) $(XLDIRS) $(XLIBS) -lm

obstacle_c_static: obstacle_c.cpp cWrapper_Path.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(STATIC_PATHLIB) $(XLDIRS) $(XLIBS) -lm

test:
	./obstacle_c_shared 6
	./obstacle_c_static 6
	@echo "-------------------------------------------------------------"
	@echo "---- demo-size test passed: trying larger instances next ----"
	@echo "-------------------------------------------------------------"
	./obstacle_c_shared 20
	./obstacle_c_static 20
