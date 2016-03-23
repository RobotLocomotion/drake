*******
Windows
*******

Note: If you want to use the MATLAB examples, these instructions assume that you have already followed the instructions `here <from_source>`_ to setup MATLAB, call ``mex -setup`` to set up a compatible compiler ( `Microsoft Visual Studio 2015 Community <http://visualstudio.com>`_ on windows, and `make sure you install the VC++ component <https://msdn.microsoft.com/en-us/library/60k1461a.aspx>`_ ), to add MATLAB to your system path, and to clone drake.


First install any missing prerequisites:

* `Java's JDK <http://www.oracle.com/technetwork/java/javase/downloads/>`_ and add the executable directory, e.g. ``C:\Program Files\Java\jdk1.7.0_51\bin`` to your `system path <http://www.java.com/en/download/help/path.xml>`_.
* `CMake <http://www.cmake.org/cmake/resources/software.html>`_ (binary version). Say yes when the CMake installer offers to add itself to your system path.
* GNU Win32 Apps (all have easy installers): `make <http://gnuwin32.sourceforge.net/packages/make.htm>`_.  Be sure to add them to your system path -- for me they all installed into ``C:\Program Files (x86)\GnuWin32\bin``
* `pkg-config-lite <http://sourceforge.net/projects/pkgconfiglite/files/>`_ or another pkg-config win32 implementation of your choosing.  Be sure to add ``[...]\pkg-config-lite-0.28-1\bin`` to your path.


If you have installed the 64-bit version of MATLAB then you must tell CMAKE to build drake as a 64-bit library.  Create a new system environment variable ``CMAKE_FLAGS`` and set it to, e.g. ``-G "Visual Studio 14 2015 Win64"``.

Drake will locally build and install the remaining prerequisites.  Open up a shell (the standard ``cmd`` shell will definitely work) and run::

	cd drake-distro
	make options    # select the externals you'd like (note some require licenses)

Make sure you let cmake (c)onfigure and (g)enerate before you exit the options gui.


When you're done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.



Other handy notes
=================

To compile Fortran pods (e.g. avl, xfoil, ...), I installed Intel Parallel Studio XE Cluster (for ifort compiler, requires a license).
Would have loved to use gfortran, but my understanding is that it is fundamentally not compatible with the rest of the visual studio building tools.


To disable the debug question everytime matlab crashes running unit tests
From a DOS prompt::

	reg add "HKCU\Software\Microsoft\Windows\Windows Error Reporting\ExcludedApplications" /v "MATLAB.exe" /t REG_SZ /d 1 /f
	reg add "HKCU\Software\Microsoft\Windows\Windows Error Reporting" /v "DontShowUI" /t REG_DWORD /d 1 /f
