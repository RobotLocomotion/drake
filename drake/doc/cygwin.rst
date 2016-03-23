**********************
Windows (using Cygwin)
**********************

Note: If you want to use the MATLAB examples, these instructions assume that you have already followed the instructions `here <from_source>`_ to setup MATLAB, call ``mex -setup`` to set up a compatible compiler ( `Microsoft Visual Studio 2015 Community <http://visualstudio.com>`_ on windows, and `make sure you install the VC++ component <https://msdn.microsoft.com/en-us/library/60k1461a.aspx>`_ ), to add MATLAB to your system path, and to clone drake.

First install any missing prerequisites:

* `Java SE Development Kit <http://www.oracle.com/technetwork/java/javase/downloads/>`_ and add the executable directory, e.g. ``C:\Program Files\Java\jdk1.7.0_51\bin`` to your `system path <http://www.java.com/en/download/help/path.xml>`_.
* `CMake <https://cmake.org/download/>`_ (binary version). Say yes when the CMake installer offers to add itself to your system path.

Download the `Cygwin installation executable <http://www.cygwin.com/install.html>`_, and save it to a file named cygwin-setup.exe somewhere in your path (for instance ``C:\cygwin64\usr\local\bin``).

Startup a new Cygwin shell with administrator privileges (using a right-click) and install the prerequisites::

    cygwin-setup -q -P autoconf automake bison ccache cmake doxygen flex git \
      gcc-g++ gcc-gfortran graphviz libjpeg-devel libgtk2.0-devel \
      libmpfr-devel libpng-devel libtool make patch perl perl-libwww-perl \
      perl-TermReadKey pkg-config python subversion swig wget
    wget -q -O - https://bootstrap.pypa.io/get-pip.py | python
    pip install -U cpplint Sphinx

If you have installed the 64-bit version of MATLAB then you must tell CMake to build Drake as a 64-bit library. Create a new system environment variable ``CMAKE_FLAGS`` and set it to, e.g. ``-G "Visual Studio 14 2015 Win64"``.

Open up a shell (the standard ``cmd`` shell will definitely work) and download the external dependencies::

    cd drake-distro
    make options  # use the GUI to choose which externals you want, then run generate before exiting
    make download-all

Setting up your shell environment
---------------------------------

Edit (or create) a ``~/.bashrc`` file and add the following lines to the bottom::

    if [ "$SSH_TTY" ]; then
      export PROCESSOR_ARCHITECTURE="AMD64"
    fi

Note: You should update the `PROCESSOR_ARCHITECTURE` for your system (you can find it by running ``cmd.exe`` and typing ``echo %PROCESSOR_ARCHITECTURE%``).  This is a standard environment variable that is not passed through to SSH by default.

---------

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
