
# Note: make will automatically delete intermediate files (google "make chains of implicit rules")

CC = gcc
CXX = g++
PKG_CONFIG_PATH := ${PKG_CONFIG_PATH}:$(shell pwd)/thirdParty

LCMFILES = $(shell find . -iname "*.lcm" | tr "\n" " " | sed "s|\./||g")
SUBDIRS:=$(shell grep -v "^\#" tobuild.txt)

LCM_CFILES = $(LCMFILES:%.lcm=%.c) 
LCM_CFLAGS = $(shell pkg-config --cflags lcm)
LCM_LDFLAGS = $(shell pkg-config --libs lcm)
EIGEN_CFLAGS = $(shell pkg-config --cflags eigen3)

#CFILES = $(LCM_CFILES)
LCM_JAVAFILES = $(LCMFILES:%.lcm=%.java)
OTHER_JAVAFILES = util/MyLCMTypeDatabase.java util/MessageMonitor.java util/CoordinateFrameData.java util/LCMCoder.java util/Transform.java systems/trajectories/JavaPP.java
JAVAFILES = $(LCM_JAVAFILES) $(OTHER_JAVAFILES)

OBJFILES = $(LCM_CFILES:%.c=%.o) 
CLASSFILES = $(JAVAFILES:%.java=%.class) 
EXTRACLASSFILES = util/MyLCMTypeDatabase*MyClassVisitor.class

MATLAB = $(shell which matlab)

all: matlab_config java c
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir all || exit 2; \
	done

debug: CXX += -g
debug: CC += -g
debug: matlab_config java c
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir debug || exit 2; \
	done

java : drake.jar

c : drake.a 

matlab_config : util/drake_config.mat .matlabroot

util/drake_config.mat .matlabroot : configure.m
	matlab -nosplash -nodesktop -r "configure;exit";
# todo: consider using configure(struct('autoconfig',true))

drake.jar : $(CLASSFILES)
	cd ..; jar -cf drake/drake.jar $(CLASSFILES:%=drake/%) $(EXTRACLASSFILES:%=drake/%)

drake.a : $(OBJFILES)
	ar rc $@ $^

.INTERMEDIATE : $(OBJFILES) $(CLASSFILES)
.PRECIOUS : $(LCMFILES) $(OTHER_JAVAFILES)

util/LCMCoder.class : util/LCMCoder.java util/CoordinateFrameData.class
	javac $< -cp $(CLASSPATH):$(shell pwd)/../

%.class : %.java
	javac $<

%.o : %.c
	$(CC) -c -I include/ $< -o $@ $(LCM_CFLAGS) $(EIGEN_CFLAGS)

%.c : %.lcm
	@if grep -i package $< ; then echo "\n *** ERROR: $< has a package specified.  Don't do that. *** \n"; exit 1; fi
	lcm-gen -c --c-cpath="$(shell echo $< | sed "s|/[A-Za-z0-9_]*\.lcm|/|")" --c-hpath="include/" $< 

%.java : %.lcm
	@if grep -i package $< ; then echo "\n *** ERROR: $< has a package specified.  Don't do that. *** \n"; exit 1; fi
	lcm-gen -j --jdefaultpkg="drake.$(shell echo $< | sed "s|/[A-Za-z0-9._]*\.lcm||g" | tr "/" ".")" --jpath=".." $<

clean : 
	-rm -f drake.jar drake.a $(LIBS) $(LCM_HFILES) $(LCM_CFILES) $(OBJFILES) $(LCM_JAVAFILES) $(CLASSFILES) $(EXTRACLASSFILES)
	@for subdir in $(SUBDIRS); do \
		echo "\n-------------------------------------------"; \
		echo "-- $$subdir"; \
		echo "-------------------------------------------"; \
		$(MAKE) -C $$subdir clean; \
	done

