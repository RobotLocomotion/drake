
# Note: I'm expecting this file to be included in just about any directory that has lcmtypes to build.  Use it like this:
#   PACKAGE = robotlib.examples.Pendulum
#   include ../../shared/lcmtypes.makefile

# Note 2: make will automatically delete intermediate files (google "make chains of implicit rules")

PACKAGEDIR = $(subst .,/,$(PACKAGE))
PACKAGEDIRLIST = $(subst ., ,$(PACKAGE))

LCMFILES = $(wildcard *.lcm)
HFILES = $(LCMFILES:%.lcm=%.h)
CFILES = $(LCMFILES:%.lcm=%.c)
OBJFILES = $(LCMFILES:%.lcm=%.o)
JAVAFILES = $(LCMFILES:%.lcm=%.java)
CLASSFILES = $(JAVAFILES:%.java=%.class)

LIBS = $(PACKAGE).a

all: $(LIBS) $(CLASSFILES)

java : $(CLASSFILES)

c : $(LIBS)

.INTERMEDIATE : $(OBJFILES)
.PRECIOUS : $(LCMFILES) $(JAVAFILES)

%.class : %.java
	javac $<

$(PACKAGE).a : $(OBJFILES)
	ar rc $@ $^

%.o : %.c
	gcc -c $<

%.c : %.lcm
	-lcm-gen -c $<

%.java : %.lcm
	-lcm-gen -j --jpath="." --jdefaultpkg="$(PACKAGE)" $<
	mv $(PACKAGEDIR)/$@ .
	rm -rf $(firstword $(PACKAGEDIRLIST))

clean : 
	-rm -f $(LIBS) $(HFILES) $(CFILES) $(OBJFILES) $(JAVAFILES) $(CLASSFILES)

