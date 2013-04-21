
# Note: make will automatically delete intermediate files (google "make chains of implicit rules")

LCMFILES = $(shell find . -iname "*.lcm" | tr "\n" " " | sed "s|\./||g")

LCM_CFILES = $(LCMFILES:%.lcm=%.c) 
LCM_CFLAGS = $(shell pkg-config --cflags lcm)
LCM_LDFLAGS = $(shell pkg-config --libs lcm)
CFILES = $(LCM_CFILES)
LCM_JAVAFILES = $(LCMFILES:%.lcm=%.java)
OTHER_JAVAFILES = util/MyLCMTypeDatabase.java util/MessageMonitor.java util/CoordinateFrameData.java util/LCMCoder.java
JAVAFILES = $(LCM_JAVAFILES) $(OTHER_JAVAFILES)

OBJFILES = $(CFILES:%.c=%.o)
CLASSFILES = $(JAVAFILES:%.java=%.class) 
EXTRACLASSFILES = util/MyLCMTypeDatabase*MyClassVisitor.class

all: java c

java : drake.jar

c : drake.a

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
	gcc -c -I include/ $< -o $@ $(LCM_CFLAGS)

%.c : %.lcm
	@if grep -i package $< ; then echo "\n *** ERROR: $< has a package specified.  Don't do that. *** \n"; exit 1; fi
	lcm-gen -c --c-cpath="$(shell echo $< | sed "s|/[A-Za-z0-9_]*\.lcm|/|")" --c-hpath="include/" $< 

%.java : %.lcm
	@if grep -i package $< ; then echo "\n *** ERROR: $< has a package specified.  Don't do that. *** \n"; exit 1; fi
	lcm-gen -j --jdefaultpkg="drake.$(shell echo $< | sed "s|/[A-Za-z0-9._]*\.lcm||g" | tr "/" ".")" --jpath=".." $<

clean : 
	-rm -f drake.jar drake.a $(LIBS) $(LCM_HFILES) $(LCM_CFILES) $(OBJFILES) $(LCM_JAVAFILES) $(CLASSFILES) $(EXTRACLASSFILES)

