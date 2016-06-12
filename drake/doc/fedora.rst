*******************************************************************
CentOS 7.2 / Red Hat Enterprise Linux 7.2 / Fedora 23 (Unsupported)
*******************************************************************

Install the prerequisites::

    sudo yum update
    sudo yum install redhat-lsb-core wget

    # CentOS
    sudo yum install centos-release-scl epel-release

    # Red Hat Enterprise Linux
    sudo yum-config-manager --enable rhel-7-server-extras-rpms \
      rhel-7-server-optional-rpms rhel-server-rhscl-7-rpms
    wget -q https://dl.fedoraproject.org/pub/epel/epel-release-latest-7.noarch.rpm
    sudo rpm -U epel-release-latest-7.noarch.rpm

    sudo yum install autoconf automake bison clang cmake doxygen flex \
      freeglut-devel gcc-c++ gcc-gfortran git graphviz gtk2-devel \
      java-*-openjdk-devel libjpeg-turbo-devel libpng-devel libtool make \
      mpfr-devel mpich numpy perl perl-HTML-Form perl-libwww-perl \
      perl-TermReadKey pkgconfig pygtk2 python-beautifulsoup4 python-devel \
      python-html5lib python-pip python-sphinx qt-devel qwt-devel subversion \
      swig unzip valgrind vtk-devel

    # CentOS and Red Hat Enterprise Linux
    sudo yum install devtoolset-3-gcc-c++ devtoolset-3-gcc-gfortran \
      devtoolset-3-valgrind
    scl enable devtoolset-3 bash

    sudo pip install -U cpplint

On CentOS and Red Hat Enterprise Linux, add the line::

    source /opt/rh/devtoolset-3/enable

to your ``.bash_profile`` or ``.bashrc``.

Download the external dependencies::

    cd drake-distro
    make options  # use the GUI to choose which externals you want, then press 'c' to configure, then 'g' to generate makefiles and exit
    make download-all

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
