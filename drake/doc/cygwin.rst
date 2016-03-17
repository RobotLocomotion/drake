**********************
Windows (using Cygwin)
**********************

Note: If you want to use the MATLAB examples, these instructions assume that you have already followed the instructions `here <from_source>`_ to setup MATLAB, call ``mex -setup`` to set up a compatible compiler ( `Microsoft Visual Studio 2015 Community <http://visualstudio.com>`_ on windows, and `make sure you install the VC++ component <https://msdn.microsoft.com/en-us/library/60k1461a.aspx>`_ ), to add MATLAB to your system path, and to clone drake.


First install any missing prerequisites:

* `Java's JDK <http://www.oracle.com/technetwork/java/javase/downloads/>`_ and add the executable directory, e.g. ``C:\Program Files\Java\jdk1.7.0_51\bin`` to your `system path <http://www.java.com/en/download/help/path.xml>`_.
* `CMake <http://www.cmake.org/cmake/resources/software.html>`_ (binary version). Say yes when the CMake installer offers to add itself to your system path.

If you have installed the 64-bit version of MATLAB then you must tell CMAKE to build drake as a 64-bit library.  Create a new system environment variable ``CMAKE_FLAGS`` and set it to, e.g. ``-G "Visual Studio 14 2015 Win64"``.

Drake will locally build and install the remaining prerequisites.  Open up a shell (the standard ``cmd`` shell will definitely work) and run::

	cd drake-distro
	make options  # use the GUI to choose which externals you want, then run generate before exiting
	make download-all

We have attempted to automate the installation of the remaining prerequisites for you, using the cygwin installer.  To use this, simply:

* download the `Cygwin installation executable <http://www.cygwin.com/install.html>`_, and save it to a file named cygwin-setup.exe somewhere in your path (for instance ``C:\cygwin64\usr\local\bin``)
* startup a new cygwin shell with administrator privileges (using a right-click on windows 8).  Then run::

	cd drake-distro
	./install_prereqs.sh cygwin

to install pod-specific dependencies using the cygwin installer.


Setting up your shell environment
---------------------------------

Edit (or create) a ``~/.bashrc`` file and add the following lines to the bottom::

	if [ "$SSH_TTY" ]; then
		export PROCESSOR_ARCHITECTURE="AMD64"
	fi


Note: You should update the `PROCESS_ARCHITECTURE` for your system (you can find it by running ``cmd.exe`` and typing ``echo %PROCESSOR_ARCHITECTURE%``).  This is a standard environment variable that is not passed through to SSH by default.

---------

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.




