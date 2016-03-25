***********************************************
Ubuntu 12.04 LTS (Precise) / 14.04 LTS (Trusty)
***********************************************

Install the prerequisites::

    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install --no-install-recommends lsb-core python-software-properties wget
    sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
    wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | sudo apt-key add -

    # Ubuntu 12.04 LTS (Precise)
    sudo add-apt-repository -y "deb http://llvm.org/apt/precise/ llvm-toolchain-precise-3.7 main"
    sudo add-apt-repository -y ppa:kalakris/cmake

    # Ubuntu 14.04 LTS (Trusty)
    sudo add-apt-repository -y "deb http://llvm.org/apt/trusty/ llvm-toolchain-trusty-3.7 main"

    sudo apt-get update
    sudo apt-get install --no-install-recommends autoconf automake bison \
      ccache clang-3.7 cmake default-jdk doxygen flex freeglut3-dev \
      g++-multilib g++-4.9-multilib gfortran gfortran-4.9 git graphviz \
      libgtk2.0-dev libhtml-form-perl libjpeg-dev libmpfr-dev libwww-perl \
      libpng-dev libqt4-dev libqt4-opengl-dev libterm-readkey-perl libtool \
      libvtk-java libvtk5-dev libvtk5-qt4-dev make mpich2 perl pkg-config \
      python-dev python-gtk2 python-numpy python-pip python-vtk subversion \
      swig valgrind

    # Ubuntu 14.04 LTS (Trusty)
    sudo apt-get install cmake-curses-gui

    sudo pip install -U -r requirements.txt

Download the external dependencies::

    cd drake-distro
    make options  # use the GUI to choose which externals you want, then press 'c' to configure, then 'g' to generate makefiles and exit
    make download-all

The version of the standard C++ libraries that are shipped with the Linux distribution of MATLAB is severely outdated and can cause problems when running mex files that are built against a newer version of the standard.  The typical error message in this case reports "Invalid MEX-Files"

To work around this issue, the symbolic link for the standard C++ library provided by MATLAB must be redirected to point to a more up-to-date version.

Update the symbolic link in MATLAB to point to the version that was installed earlier into ``/usr/lib``.  An example for MATLAB R2015b is shown below::

    cd /usr/local/MATLAB/R2015b/sys/os/glnxa64
    sudo rm libstdc++.so.6
    sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.9/libstdc++.so libstdc++.so.6

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
