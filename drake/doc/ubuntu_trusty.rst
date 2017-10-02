.. _build_from_source_trusty:

*************************
Ubuntu 14.04 LTS (Trusty)
*************************

The following instructions are written for Ubuntu 14.04 LTS, which is a
supported Drake platform.

Prerequisite setup is automated. Simply run::

    sudo ./setup/ubuntu/14.04/install_prereqs.sh

You may need to respond to interactive prompts to confirm that you agree to add
various `apt` repositories to your system and that you agree to the license
conditions of certain software therein.

Using the Legacy CMake Build System
===================================

To use the legacy CMake build, you must also complete the following steps.

.. _cmake:

CMake
-----

CMake 3.5.2 or higher is required. Visit the `CMake Download Page`_ to obtain
the CMake 3.5.2 pre-compiled binaries.  Extract the archive and add its ``bin``
directory to the ``PATH`` environment variable. For example, below is a
suggested sequence of commands that installs CMake 3.5.2 into `~/tools/` and
then modifies `~/.bashrc` with the new ``PATH`` environment variable::

    mkdir -p ~/tools
    cd ~/tools
    wget https://cmake.org/files/v3.5/cmake-3.5.2-Linux-x86_64.tar.gz
    tar zxvf cmake-3.5.2-Linux-x86_64.tar.gz
    rm cmake-3.5.2-Linux-x86_64.tar.gz
    cd cmake-3.5.2-Linux-x86_64/bin
    echo "export PATH=`pwd`:\$PATH" >> ~/.bashrc

.. _`CMake Download Page`: https://cmake.org/download/

Environment
-----------

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

    export CC=clang-3.9 CXX=clang++-3.9 FC=gfortran-4.9

Or precede the initial call to ``cmake`` with compiler specifications.
For example::

    env CC=clang-3.9 CXX=clang++-3.9 FC=gfortran-4.9 cmake ...

MATLAB
======

The version of the standard C++ libraries that are shipped with the Linux distribution of MATLAB is severely outdated and can cause problems when running mex files that are built against a newer version of the standard.  The typical error message in this case reports "Invalid MEX-Files"

To work around this issue, the symbolic link for the standard C++ library provided by MATLAB must be redirected to point to a more up-to-date version.

Update the symbolic link in MATLAB to point to the version that was installed earlier into ``/usr/lib``.  An example for MATLAB R2016a is shown below::

    cd /usr/local/MATLAB/R2017a/sys/os/glnxa64
    sudo rm libstdc++.so.6
    sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.9/libstdc++.so libstdc++.so.6
