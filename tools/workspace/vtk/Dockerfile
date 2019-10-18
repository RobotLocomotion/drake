# This Dockerfile and the accompanying shell scripts are used by the project
# maintainers to create the precompiled vtk binaries that are downloaded during
# the build. They are neither called during the build nor expected to be called
# by most developers or users of the project.

ARG CODENAME=bionic
FROM ubuntu:$CODENAME
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
    ca-certificates \
    cmake \
    g++ \
    gcc \
    git \
    libexpat1-dev \
    libfreetype6-dev \
    libgl1-mesa-dev \
    libglib2.0-dev \
    libhdf5-dev \
    libjpeg8-dev \
    libjsoncpp-dev \
    liblz4-dev \
    libnetcdf-dev \
    libogg-dev \
    libpng-dev \
    libqt5x11extras5-dev \
    libtbb-dev \
    libtheora-dev \
    libtiff5-dev \
    libxml2-dev \
    libxt-dev \
    lsb-release \
    ninja-build \
    python-dev \
    python3-dev \
    qt5-default \
    qttools5-dev \
    wget \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*
RUN wget --quiet https://downloads.sourceforge.net/project/ispcmirror/v1.10.0/ispc-v1.10.0-linux.tar.gz \
  && tar -xzf ispc-*-linux.tar.gz \
  && cp -f ispc-*-Linux/bin/ispc /usr/local/bin \
  && rm -rf ispc-*-Linux ispc-*-linux.tar.gz
COPY dockerfile_build_embree_binaries /dockerfile_build_embree_binaries
RUN git clone --branch v3.5.1 --config advice.detachedHead=false --depth 1 --quiet https://github.com/embree/embree.git \
  && ./dockerfile_build_embree_binaries \
  && rm -rf dockerfile_build_embree_binaries embree
COPY dockerfile_build_ospray_binaries /dockerfile_build_ospray_binaries
COPY ospray_no_fast_math.patch /ospray_no_fast_math.patch
RUN git clone --branch v1.8.2 --config advice.detachedHead=false --depth 1 --quiet https://github.com/ospray/ospray.git \
  && cd ospray \
  && git apply ../ospray_no_fast_math.patch \
  && cd .. \
  && rm -f ospray_no_fast_math.patch \
  && ./dockerfile_build_ospray_binaries \
  && rm -rf dockerfile_build_ospray_binaries ospray
COPY dockerfile_build_vtk_binaries /dockerfile_build_vtk_binaries
COPY vtk_rendering_ospray.patch /vtk_rendering_ospray.patch
RUN git clone --branch v8.2.0 --config advice.detachedHead=false --depth 1 --quiet https://gitlab.kitware.com/vtk/vtk.git \
  && cd vtk \
  && git apply ../vtk_rendering_ospray.patch \
  && cd .. \
  && rm -f vtk_rendering_ospray.patch \
  && ./dockerfile_build_vtk_binaries /usr/bin/python3 \
  && ./dockerfile_build_vtk_binaries /usr/bin/python2 \
  && rm -rf dockerfile_build_vtk_binaries vtk \
  && sed -i 's|VTK_RUNTIME_DIRS "/opt/vtk/bin"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/bin"|g' /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake \
  && sed -i 's|VTK_RUNTIME_DIRS "/opt/vtk/lib"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/lib"|g' /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake \
  && sed -i 's|/opt/vtk|${_IMPORT_PREFIX}|g' /opt/vtk/lib/cmake/vtk-*/VTKTargets.cmake \
  && sed -i 's|${_IMPORT_PREFIX}/include/ospray;${_IMPORT_PREFIX}/include/ospray/SDK|${_IMPORT_PREFIX}/include/ospray|g' /opt/vtk/lib/cmake/vtk-*/VTKTargets.cmake \
  && sed -i 's|VTK_PYTHONPATH "/opt/vtk/lib/python2.7/site-packages"|VTK_PYTHONPATH "${VTK_INSTALL_PREFIX}/lib/python2.7/site-packages"|g' /opt/vtk/lib/cmake/vtk-*/Modules/vtkPython.cmake
RUN cd /opt/vtk \
  && tar -czf vtk-8.2.0-embree-3.5.1-ospray-1.8.2-python-$(python2 --version 2>&1 | sed -n -e 's/^.*\n*Python \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-python-$(python3 --version | sed -n -e 's/^.*\n*Python \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-qt-$(qmake --version | sed -n -e 's/^.*\n*Qt version \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-$(lsb_release --codename --short)-x86_64-4.tar.gz *
