*************************
Ubuntu 14.04 LTS (Trusty)
*************************

The following instructions are written for Ubuntu 14.04 LTS, which is a
supported Drake platform.

Install Prerequisites
=====================

C++ Compiler
------------

A compiler supporting C++14 or higher is required. Choose Clang or GCC:

GCC 4.9::

    sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install g++-4.9-multilib gfortran-4.9

Clang 3.7::

    sudo apt-get install --no-install-recommends lsb-core software-properties-common wget
    wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | sudo apt-key add -
    sudo add-apt-repository -y "deb http://llvm.org/apt/trusty/ llvm-toolchain-trusty-3.7 main"
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install clang-3.7

.. _cmake:

CMake
-----

CMake 3.5 or higher is required. Visit the `CMake Download Page`_ to obtain
the CMake 3.5 pre-compiled binaries.  Extract the archive and add its ``bin``
directory to the ``PATH`` environment variable. For example, below is a
suggested sequence of commands that installs CMake 3.5 into `~/tools/` and then
modifies `~/.bashrc` with the new ``PATH`` environment variable::

    mkdir -p ~/tools
    cd ~/tools
    wget https://cmake.org/files/v3.5/cmake-3.5.2-Linux-x86_64.tar.gz
    tar zxvf cmake-3.5.2-Linux-x86_64.tar.gz
    rm cmake-3.5.2-Linux-x86_64.tar.gz
    cd cmake-3.5.2-Linux-x86_64/bin
    echo "export PATH=`pwd`:\$PATH" >> ~/.bashrc

.. _`CMake Download Page`: https://cmake.org/download/

Other Prerequisites
-------------------

Other prerequisites may be installed as follows::

    sudo apt-get update
    sudo apt-get install --no-install-recommends autoconf automake bison \
      default-jdk doxygen freeglut3-dev git graphviz libgtk2.0-dev \
      libhtml-form-perl libjpeg-dev libmpfr-dev libpng-dev libqt4-dev \
      libqt4-opengl-dev libqwt-dev libterm-readkey-perl libtool libvtk-java \
      libvtk5-dev libvtk5-qt4-dev libwww-perl make ninja-build perl pkg-config \
      python-bs4 python-dev python-gtk2 python-html5lib python-numpy
      python-pip python-sphinx python-vtk python-yaml unzip valgrind

Environment
-----------

There are up to two important environment variables to set depending on which
version of Ubuntu is being used.

Compiler Environment Variables
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Since Drake does not use the system default compiler, the desired compiler
must be manually specified. One way to do this is to set the ``CC``, ``CXX``,
and ``FC``, environment variables. This can be done by executing the command
below. To avoid needing to run this command each time a new terminal is opened,
the command below can also be added to the ``~/.bashrc`` file::

    export CC=gcc-4.9 CXX=g++-4.9 FC=gfortran-4.9

Alternatively, the initial call to ``cmake`` can be preceded with
environment variable settings that specify the correct compiler. For example::

    env CC=gcc-4.9 CXX=g++-4.9 FC=gfortran-4.9 cmake ...

The above examples result in the use of ``gcc`` as the compiler. If you want to
use ``clang`` as the compiler, place the following in your ``~/.bashrc`` file::

    export CC=clang-3.7 CXX=clang++-3.7 FC=gfortran-4.9

Or precede the initial call to ``cmake`` with compiler specifications.
For example::

    env CC=clang-3.7 CXX=clang++-3.7 FC=gfortran-4.9 cmake ...

MATLAB
======

The version of the standard C++ libraries that are shipped with the Linux distribution of MATLAB is severely outdated and can cause problems when running mex files that are built against a newer version of the standard.  The typical error message in this case reports "Invalid MEX-Files"

To work around this issue, the symbolic link for the standard C++ library provided by MATLAB must be redirected to point to a more up-to-date version.

Update the symbolic link in MATLAB to point to the version that was installed earlier into ``/usr/lib``.  An example for MATLAB R2016a is shown below::

    cd /usr/local/MATLAB/R2016a/sys/os/glnxa64
    sudo rm libstdc++.so.6
    sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.9/libstdc++.so libstdc++.so.6

Return to Generic Instructions
==============================

When these platform-specific steps are completed,
return to :doc:`from_source` to complete and test the installation.

Note that, from now on, when running Drake commands (including the
ones in the linked instructions, such as ``make`` or ``make test``),
proper `Environment`_ must always be established.
