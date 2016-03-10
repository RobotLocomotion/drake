# -*- Makefile -*-

.PHONY: all test

PATHDIR = ../..
PATHLIB = $(PATHDIR)/lib

# additional libs - we expect them to be required only for static builds
XLIBS = -lgfortran

CXX    = g++
CXXFLAGS = -I$(PATHDIR)/include -m64

all: obstacle_c_shared obstacle_c_static

obstacle_c_shared: obstacle_c.cpp cWrapper_Path.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^         -L$(PATHLIB) -lpath47 -lm

obstacle_c_static: obstacle_c.cpp cWrapper_Path.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(PATHLIB)/libpath47.a $(XLIBS)

test:
	./obstacle_c_shared 6
	./obstacle_c_static 6
	@echo "-------------------------------------------------------------"
	@echo "---- demo-size test passed: trying larger instances next ----"
	@echo "-------------------------------------------------------------"
	./obstacle_c_shared 20
	./obstacle_c_static 20
