
# Note: make will automatically delete intermediate files (google "make chains of implicit rules")

LCMFILES = $(shell find . -iname "*.lcm" | tr "\n" " " | sed "s|\./||g")

LCM_CFILES = $(LCMFILES:%.lcm=%.c) 
CFILES = $(LCM_CFILES)
LCM_JAVAFILES = $(LCMFILES:%.lcm=%.java)
OTHER_JAVAFILES = util/MyLCMTypeDatabase.java
JAVAFILES = $(LCM_JAVAFILES) $(OTHER_JAVAFILES)

OBJFILES = $(CFILES:%.c=%.o)
CLASSFILES = $(JAVAFILES:%.java=%.class) 
EXTRACLASSFILES = util/MyLCMTypeDatabase*MyClassVisitor.class

all: java c

java : robotlib.jar

c : robotlib.a

robotlib.jar : $(CLASSFILES)
	cd ..; jar -cf robotlib/robotlib.jar $(CLASSFILES:%=robotlib/%) $(EXTRACLASSFILES:%=robotlib/%)

robotlib.a : $(OBJFILES)
	ar rc $@ $^

.INTERMEDIATE : $(OBJFILES) $(CLASSFILES)
.PRECIOUS : $(LCMFILES) $(OTHER_JAVAFILES)

%.class : %.java
	javac $<

%.o : %.c
	gcc -c -I include/ $< -o $@

%.c : %.lcm
	@if grep -i package $< ; then echo "\n *** ERROR: $< has a package specified.  Don't do that. *** \n"; exit 1; fi
	lcm-gen -c --c-cpath="$(shell echo $< | sed "s|/[A-Za-z0-9_]*\.lcm|/|")" --c-hpath="include/" $<

%.java : %.lcm
	@if grep -i package $< ; then echo "\n *** ERROR: $< has a package specified.  Don't do that. *** \n"; exit 1; fi
	lcm-gen -j --jdefaultpkg="robotlib.$(shell echo $< | sed "s|/[A-Za-z0-9._]*\.lcm||g" | tr "/" ".")" --jpath=".." $<

clean : 
	-rm -f robotlib.jar robotlib.a $(LIBS) $(LCM_HFILES) $(LCM_CFILES) $(OBJFILES) $(LCM_JAVAFILES) $(CLASSFILES) $(EXTRACLASSFILES)

