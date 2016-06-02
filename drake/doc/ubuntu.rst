*************************
Ubuntu 14.04 LTS (Trusty)
*************************

The following instructions are written for the officially supported version of
Ubuntu:

* Ubuntu 14.04 LTS (Trusty)

Some hints are given for newer versions of Ubuntu, but details may vary.

Install Prerequisites
=====================

C++ Compiler
------------

A compiler supporting C++11 or higher is required. The installation process
differs depending on which version of Ubuntu you’re using.

Ubuntu 15.10 (Wily)
~~~~~~~~~~~~~~~~~~~

On Ubuntu 15.10 (Wily) and higher the system compiler is sufficient::

    sudo apt-get install g++-multilib

Ubuntu 14.04 LTS (Trusty)
~~~~~~~~~~~~~~~~~~~~~~~~~

On Ubuntu 14.04 LTS (Trusty) one must install GCC 4.9 or higher, e.g.::

    sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install g++-4.9-multilib

Or, Clang 3.7::

    sudo apt-get install --no-install-recommends lsb-core software-properties-common wget
    wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | sudo apt-key add -
    sudo add-apt-repository -y "deb http://llvm.org/apt/trusty/ llvm-toolchain-trusty-3.7 main"
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install clang-3.7

.. _cmake:

CMake
-----

CMake 3.5 or higher is required. The installation process differs depending on
which version of Ubuntu you're using.

Ubuntu 16.04 LTS (Xenial)
~~~~~~~~~~~~~~~~~~~~~~~~~

On Ubuntu 16.04 LTS (Xenial) and higher one may use the system package::

    sudo apt-get install cmake cmake-curses-gui

.. _cmake_on_older_ubuntu_versions:

Older Versions of Ubuntu
~~~~~~~~~~~~~~~~~~~~~~~~

On older Ubuntu versions please visit the `CMake Download Page`_ to obtain
the CMake 3.5 pre-compiled binaries.  Extract the archive and add its ``bin``
directory to your ``PATH`` environment variable. For example, below is a
suggested sequence of commands that installs CMake 3.5 into `~/tools/` and then
modifies `~/.bashrc` with the new ``PATH`` environment variable::

    mkdir -p ~/tools
    cd ~/tools
    wget https://cmake.org/files/v3.5/cmake-3.5.2-Linux-x86_64.tar.gz
    tar zxvf cmake-3.5.2-Linux-x86_64.tar.gz
    rm cmake-3.5.2-Linux-x86_64.tar.gz
    cd cmake-3.5.2-Linux-x86_64/bin
    echo "export PATH=`pwd`:$PATH" >> ~/.bashrc

.. _`CMake Download Page`: https://cmake.org/download/

Other Prerequisites
-------------------

Other prerequisites may be installed as follows::

    sudo apt-get update
    sudo apt-get install --no-install-recommends autoconf automake bison \
      ccache default-jdk doxygen flex freeglut3-dev git \
      graphviz libgtk2.0-dev libhtml-form-perl libjpeg-dev libmpfr-dev \
      libwww-perl libpng-dev libqt4-dev libqt4-opengl-dev libqwt-dev \
      libterm-readkey-perl libtool libvtk-java libvtk5-dev libvtk5-qt4-dev \
      make mpich2 perl pkg-config python-bs4 python-dev python-gtk2 \
      python-html5lib python-numpy python-pip python-sphinx python-vtk \
      subversion swig unzip valgrind
    sudo pip install -U cpplint

Environment
-----------

If you are not using your system's default compiler (for example if you need to
use GCC/G++ 4.9 on Ubuntu 14.04 LTS), you must specify the desired compiler by
setting the ``CC`` and ``CXX`` environment variables in your terminal. This can
be done by executing the command below. To avoid needing to set these
environment variables each time you open a new terminal, you can also add the
line below to your ``~/.bashrc`` file::

    export CC=gcc-4.9 CXX=g++-4.9

Alternatively, you can precede every call to ``make`` with environment variable
settings that specify the correct compiler::

    env CC=gcc-4.9 CXX=g++-4.9 make ...

If you are not using a system-installed `CMake`_, ensure that it is in the
``PATH``::

    export PATH=/path/to/cmake-binary/bin

For more information, see :ref:`these instructions <cmake_on_older_ubuntu_versions>`.

External Source Dependencies
============================

Download the external dependencies::

    cd drake-distro
    make options
    # Use the GUI to choose which externals you want,
    # then press 'c' twice to configure,
    # then 'g' to generate makefiles and exit.
    make download-all

MATLAB
======

The version of the standard C++ libraries that are shipped with the Linux distribution of MATLAB is severely outdated and can cause problems when running mex files that are built against a newer version of the standard.  The typical error message in this case reports "Invalid MEX-Files"

To work around this issue, the symbolic link for the standard C++ library provided by MATLAB must be redirected to point to a more up-to-date version.

Update the symbolic link in MATLAB to point to the version that was installed earlier into ``/usr/lib``.  An example for MATLAB R2016a is shown below::

    cd /usr/local/MATLAB/R2016a/sys/os/glnxa64
    sudo rm libstdc++.so.6
    sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.9/libstdc++.so libstdc++.so.6

ccache
======

You may wish to use ``ccache`` to speed up your (re)builds.
To do so, add ``/usr/lib/ccache`` to the front of your ``$PATH``.

Return to Generic Instructions
==============================

When you are done with these platform-specific steps,
return to :doc:`from_source` to complete and test your installation.

Note that when you run drake commands from now on (including the
ones in the linked instructions, such as ``make`` or ``make test``),
you must always establish the proper `Environment`_.
