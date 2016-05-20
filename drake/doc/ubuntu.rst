*************************
Ubuntu 14.04 LTS (Trusty)
*************************

Install the prerequisites::

    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install --no-install-recommends lsb-core software-properties-common wget
    sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
    wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | sudo apt-key add -

    sudo add-apt-repository -y "deb http://llvm.org/apt/trusty/ llvm-toolchain-trusty-3.7 main"

    sudo apt-get update
    sudo apt-get install --no-install-recommends autoconf automake bison \
      ccache clang-3.7 cmake cmake-curses-gui default-jdk doxygen flex \
      freeglut3-dev g++-multilib g++-4.9-multilib gfortran gfortran-4.9 git \
      graphviz libgtk2.0-dev libhtml-form-perl libjpeg-dev libmpfr-dev \
      libwww-perl libpng-dev libqt4-dev libqt4-opengl-dev libqwt-dev \
      libterm-readkey-perl libtool libvtk-java libvtk5-dev libvtk5-qt4-dev \
      make mpich2 perl pkg-config python-bs4 python-dev python-gtk2 \
      python-html5lib python-numpy python-pip python-sphinx python-vtk \
      subversion swig unzip valgrind
    sudo pip install -U cpplint

Download the external dependencies::

    cd drake-distro
    env CXX=g++-4.9 CC=gcc-4.9 make options
    # Use the GUI to choose which externals you want, then press 'c' twice to configure, then 'g' to generate makefiles and exit.
    env CXX=g++-4.9 CC=gcc-4.9 make download-all

The version of the standard C++ libraries that are shipped with the Linux distribution of MATLAB is severely outdated and can cause problems when running mex files that are built against a newer version of the standard.  The typical error message in this case reports "Invalid MEX-Files"

To work around this issue, the symbolic link for the standard C++ library provided by MATLAB must be redirected to point to a more up-to-date version.

Update the symbolic link in MATLAB to point to the version that was installed earlier into ``/usr/lib``.  An example for MATLAB R2016a is shown below::

    cd /usr/local/MATLAB/R2016a/sys/os/glnxa64
    sudo rm libstdc++.so.6
    sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.9/libstdc++.so libstdc++.so.6

You may wish to use `ccache` to speed up your (re)builds.
To do so, add `/usr/lib/ccache` to the front of your `$PATH`.

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.

Note that when you run drake commands from now on (including the
ones in the linked instructions, such as `make` or `make test`),
you must always precede them with `env CXX=g++-4.9 CC=gcc-4.9`,
just like you did in the the `make options` step above).
