
# Note: make will automatically delete intermediate files (google "make chains of implicit rules")

LCMFILES = $(shell find . -iname "*.lcm" | tr "\n" " " | sed "s|\./||g")

LCM_CFILES = $(LCMFILES:%.lcm=%.c) #$(shell echo $(LCMFILES) | sed "s|$<[A-Za-z0-9._/]*/|src/|g" | sed "s|\.lcm$<|.c|g")
LCM_OBJFILES = $(LCM_CFILES:%.c=%.o)
LCM_JAVAFILES = $(LCMFILES:%.lcm=%.java)
LCM_CLASSFILES = $(JAVAFILES:%.java=%.class)

#  this looks stupid and redundant, but soon i will want to make more than just LCM files
CFILES = $(LCM_CFILES)
OBJFILES = $(LCM_OBJFILES)
JAVAFILES = $(LCM_JAVAFILES)
CLASSFILES = $(LCM_CLASSFILES)

all: java c

java : robotlib.jar

c : robotlib.a
	
robotlib.jar : $(CLASSFILES)
	cd ..; jar -cf robotlib/robotlib.jar $(CLASSFILES:%=robotlib/%)

robotlib.a : $(OBJFILES)
	ar rc $@ $^

.INTERMEDIATE : $(OBJFILES) $(CLASSFILES)
.PRECIOUS : $(LCMFILES) 

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
	-rm -f robotlib.jar robotlib.a $(LIBS) $(HFILES) $(CFILES) $(OBJFILES) $(JAVAFILES) $(CLASSFILES)

