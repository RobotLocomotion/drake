**************************
Frequently Asked Questions
**************************

----------

**Q1.** When I'm using VMWare (Fusion, Workstation) I cannot get the 3D visualizer to work. How do I fix that?

**A1.** Symptom: the simulation runs and the visualization window appears, but no objects are actually drawn. This appeared to be due to display drivers and/or non support of hardware-accelerated rendering.To address this, go to ``Virtual Machine Settings``, and check the ``Accelerate 3D Graphics`` box under Display settings; now the simulations draw properly.

----------

**Q2.** I can't get my compiled Java classes to work in Drake (e.g. after running make)

**A2.** This could possibly be an issue with the version of the Java compiler (javac) installed on your system. MATLAB currently cannot run classes that have been compiled with the Java 1.7 (or free alternative compilers that are version 1.7 compliant). You should retarget your compiling for version 1.6. This can be done by passing javac the ``--source=1.6 --target=1.6`` flags.

----------

**Q3.** Error building Drake w/ MATLAB R2013a after switching to Mavericks

Symptoms: when running make in drake, you get the following error messages::

	xcodebuild: error: SDK "macosx10.7" cannot be located.
	xcrun: error: unable to find utility "clang", not a developer tool or in PATH
	-- compiler1 version string:
	-- compiler2 version string: 4.2.1
	CMake Error at cmake/mex.cmake:203 (message):
		Your cmake C compiler is: /usr/bin/cc but your mex options use: xcrun -sdk
		macosx10.7 clang .  Consider rerunning 'mex -setup' in Matlab.

**A3.** 
Cause: Matlab's mexopts.sh in the bin folder of your Matlab installation statically refers to the 10.7 sdk, which was removed in Mavericks . 

Fix: Replace all occurrences of ``10.7`` in mexopts.sh by ``10.9``. After this, run ``mex -setup`` in Matlab and select the option that mentions mexopts.sh (option 1 in my case).
http://stackoverflow.com/questions/20294160/matlab-error-regarding-compile-mex-command

----------

**Q4.** Can't compile mex after upgrading XCode to 5.1 on Mac.
   http://www.mathworks.com/matlabcentral/answers/121305-mex-cpp-under-matlab-2013b-and-xcode-5-0
   http://stackoverflow.com/questions/22367516/matlab-mex-compile-error

The error message looks like::

	/Applications/MATLAB_R2012a.app/extern/include/tmwtypes.h:819:9: error: unknown type name 'char16_t'
	typedef char16_t CHAR16_T;



**A4.** MATLAB's types are not compatible with the newest version of clang.  Hopefully they will get in sync soon, but for now I've decided the best fix is to edit the twmtypes.h file::

	/*typedef char16_t CHAR16_T;*/
	typedef UINT16_T CHAR16_T;

----------

**Q5.** Drake tells me I don't have Simulink 3D Animation Toolbox, but I'm sure that I do!

**A5.** You might have to actually tell MATLAB to install the tool, running ``vrinstall`` in MATLAB.  On windows, use ``vrinstall -install editor``.

----------

**Q6.** Undefined symbol "___sincos_stret" on Mac.  

**A6.** This is an optimization in the XCode 5.  Update your mexopts.sh to make sure your MACOSX_DEPLOYMENT_TARGET is set to 10.9.  (It's best to just search and replace 10.8 for 10.9)


----------

**Q7.** I'm having trouble trying to install drake on Ubuntu after installing NVidia drivers. I get the error::

	make[5]: *** No rule to make target `/usr/lib/x86_64-linux-gnu/libGL.so', needed by `lib/libbot2-frames-renderers.so.1'.  Stop.

**A7.** You may need to follow these steps:
http://techtidings.blogspot.com/2012/01/problem-with-libglso-on-64-bit-ubuntu.html

----------

**Q8.** Can't find ``jpeglib.h`` when compiling bot2-vis on Mac.  Or can't find ``gmp.h`` when compiling bertini.

**A8.** Make sure you've installed the xcode command line tools with ``xcode-select --install``, then ``make clean`` and ``make`` again.

----------

**Q9.** In MATLAB on OSX Yosemite, you may see the following error when calling LCM::

	"LC singleton fail: java.net.SocketException: Can't assign requested address"

**A9.**
Apply the resolution described here: https://github.com/RobotLocomotion/drake/issues/558

----------

**Q10.** ``Could not find MEX_CC using mex -v`` on Mac after updating XCode.

**A10.** Open Matlab. Run::

	edit ([matlabroot '/bin/maci64/mexopts/clang_maci64.xml'])


Search for ``MacOSX10.10``. Toward the bottom, you will find four hits.

Two of these hits (one on line 121, one on line 133 of the unmodified file) look like::

	<dirExists name="$$/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk" />

In both locations, copy this line, paste it on the next line and change the second one to "10.11", like this::

	<dirExists name="$$/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk" />
	<dirExists name="$$/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.11.sdk" />


The other two hits (one on line 123, one on line 135 of the unmodified file) look like::

	<cmdReturns name="find $$ -name MacOSX10.10.sdk" />

Repeat the copy/paste/modify 10.10 to 10.11 process for these lines.

Repeat this whole process for `clang++_maci64.xml`, `gfortran.xml`, and `intel_fortran.xml`.

(note: this is a slightly more thorough version of the resolution described here: http://www.mathworks.com/matlabcentral/answers/243868-mex-can-t-find-compiler-after-xcode-7-update-r2015b ).
