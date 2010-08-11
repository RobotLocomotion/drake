
# Note: make will automatically delete intermediate files (google "make chains of implicit rules")

#LCMFILES = $(wildcard *.lcm)

LCMFILES = shared/lcmt_scope_data.lcm \
		examples/Pendulum/lcmt_pendulum_u.lcm examples/Pendulum/lcmt_pendulum_x.lcm \
    examples/Glider/lcmt_glider_u.lcm examples/Glider/lcmt_glider_x.lcm


LCM_HFILES = $(LCMFILES:%.lcm=%.h)
LCM_CFILES = $(LCMFILES:%.lcm=%.c)
LCM_OBJFILES = $(LCMFILES:%.lcm=%.o)
LCM_JAVAFILES = $(LCMFILES:%.lcm=%.java)
LCM_CLASSFILES = $(JAVAFILES:%.java=%.class)

#  this looks stupid and redundant, but soon i will want to make more than just LCM files
HFILES = $(LCM_HFILES)
CFILES = $(LCM_CFILES)
OBJFILES = $(LCM_OBJFILES)
JAVAFILES = $(LCM_JAVAFILES)
CLASSFILES = $(LCM_CLASSFILES)

all: java 

java : robotlib.jar

c : robotlib.a

robotlib.jar : $(CLASSFILES)
	cd ..; jar -cf robotlib/robotlib.jar $(CLASSFILES:%=robotlib/%)

robotlib.a : $(OBJFILES)
	ar rc $@ $^

.INTERMEDIATE : $(OBJFILES)
.PRECIOUS : $(LCMFILES) $(JAVAFILES)

%.class : %.java
	javac $<

%.o : %.c
	gcc -c $<

%.c : %.lcm
	-lcm-gen -c $<

%.java : %.lcm
	-lcm-gen -j --jpath=".." $<

clean : 
	-rm -f $(LIBS) $(HFILES) $(CFILES) $(OBJFILES) $(JAVAFILES) $(CLASSFILES)

