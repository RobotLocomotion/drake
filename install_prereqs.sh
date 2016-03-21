#!/bin/bash

case "$1" in
  "cygwin")
    cygwin-setup -q -P autoconf automake bison ccache cmake doxygen flex git \
      gcc-g++ gcc-gfortran graphviz libjpeg-devel libgtk2.0-devel \
      libmpfr-devel libpng-devel libtool make patch perl perl-libwww-perl \
      perl-TermReadKey pkg-config python subversion swig wget
    ;;
  "fedora")
    yum install -y redhat-lsb-core wget
    distributor=$(lsb_release -is)
    if [[ "${distributor}" = "CentOS" ]]; then
      yum install -y centos-release-scl epel-release
    elif [[ "${distributor}" = "RedHatEnterpriseServer" ]]; then
      release=$(lsb_release -rs | cut -f1 -d .)
      yum-config-manager --enable "rhel-${release}-server-extras-rpms" \
        "rhel-${release}-server-optional-rpms" \
        "rhel-server-rhscl-${release}-rpms"
      # yum-config-manager --enable rhui-REGION-rhel-server-extras \
      #   rhui-REGION-rhel-server-optional \
      #   rhui-REGION-rhel-server-rhscl  # Amazon EC2
      wget -q "https://dl.fedoraproject.org/pub/epel/epel-release-latest-${release}.noarch.rpm"
      rpm -U "epel-release-latest-${release}.noarch.rpm"
    fi
    yum install -y autoconf automake bison ccache clang cmake doxygen flex \
      freeglut-devel gcc-c++ gcc-gfortran git graphviz gtk2-devel \
      java-*-openjdk-devel libjpeg-turbo-devel libpng-devel libtool make \
      mpfr-devel mpich numpy perl perl-HTML-Form perl-libwww-perl \
      perl-TermReadKey pkgconfig pygtk2 python-devel python-pip qt-devel \
      subversion swig valgrind vtk-devel
    if [[ "${distributor}" != "Fedora" ]]; then
      yum install -y devtoolset-3-gcc-c++ devtoolset-3-gcc-gfortran \
        devtoolset-3-valgrind
      echo "----------------------------------------------------------------"
      echo "  To enable GCC 4.9, call 'scl enable devtoolset-3 bash' or add"
      echo "  'source /opt/rh/devtoolset-3/enable' to your .bash_profile."
      echo "----------------------------------------------------------------"
    fi
    pip install -U -r requirements.txt
    ;;
  "homebrew")
    brew tap homebrew/python
    brew tap homebrew/science
    brew update
    brew install autoconf automake ccache cmake doxygen gcc glib graphviz gtk+ \
      jpeg libpng libtool mpfr mpich2 numpy python qt swig valgrind wget
    if [[ "${CI}" = "true" ]]; then
      brew install vtk5
    else
      brew install vtk5 --with-qt
    fi
    pip install -U -r requirements.txt
    ;;
  "macports")
    port selfupdate
    port install automake ccache cmake doxygen freeglut gcc49 glib2 graphviz \
      gtk2 jpeg libpng libtool mpfr mpich-default p5-html-form p5-libwww-perl \
      p5-term-readkey py-numpy py-pip python27 qt4-mac swig valgrind wget
    port install vtk5 +python27 +qt4_mac
    port select --set mpich mpich-mp-fortran
    pip install -U -r requirements.txt
    ;;
  "ubuntu")
    apt-get update -qq
    apt-get install -q -y --no-install-recommends lsb-core \
      python-software-properties wget
    add-apt-repository -y ppa:ubuntu-toolchain-r/test
    wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | apt-key add -
    release=$(lsb_release -cs)
    add-apt-repository -y \
      "deb http://llvm.org/apt/${release}/ llvm-toolchain-${release}-3.7 main"
    if [[ "${release}" = "precise" ]]; then
      add-apt-repository -y ppa:kalakris/cmake
    fi
    apt-get update -qq
    apt-get install -q -y --no-install-recommends autoconf automake bison \
      ccache clang-3.7 cmake default-jdk doxygen flex freeglut3-dev \
      g++-multilib g++-4.9-multilib gfortran gfortran-4.9 git graphviz \
      libgtk2.0-dev libhtml-form-perl libjpeg-dev libmpfr-dev libwww-perl \
      libpng-dev libqt4-dev libqt4-opengl-dev libterm-readkey-perl libtool \
      libvtk-java libvtk5-dev libvtk5-qt4-dev make mpich2 perl pkg-config \
      python-dev python-gtk2 python-numpy python-pip python-vtk subversion \
      swig valgrind
    if [[ "${release}" != "precise" ]]; then
      apt-get install cmake-curses-gui
    fi
    pip install -U -r requirements.txt
    ;;
  *)
    echo "Usage: ./install_prereqs.sh package_manager" >&2
    echo "where package_manager is one of the following: " >&2
    echo "  cygwin" >&2
    echo "  fedora" >&2
    echo "  homebrew" >&2
    echo "  macports" >&2
    echo "  ubuntu" >&2
    exit 1
    ;;
esac
