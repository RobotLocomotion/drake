# This Dockerfile and the accompanying shell scripts are used by the project
# maintainers to create the precompiled vtk binaries that are downloaded during
# the build. They are neither called during the build nor expected to be called
# by most developers or users of the project.

ARG UBUNTU_CODENAME=bionic
FROM ubuntu:${UBUNTU_CODENAME}
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
    ca-certificates \
    cmake \
    g++ \
    gcc \
    git \
    libeigen3-dev \
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
    python3-dev \
    qt5-default \
    qttools5-dev \
    wget \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*
RUN wget --quiet https://github.com/ispc/ispc/releases/download/v1.13.0/ispc-v1.13.0-linux.tar.gz \
  && echo '8ab1189bd5db596b3eee9d9465d3528b6626a7250675d67102761bb0d284cd21  ispc-v1.13.0-linux.tar.gz' | sha256sum -c \
  && tar -xzf ispc-*-linux.tar.gz \
  && cp -f ispc-*-linux/bin/ispc /usr/local/bin \
  && rm -rf ispc-*-linux ispc-*-linux.tar.gz
COPY dockerfile_build_embree_binaries /dockerfile_build_embree_binaries
RUN git clone --branch v3.5.2 --config advice.detachedHead=false --depth 1 --quiet https://github.com/embree/embree.git \
  && ./dockerfile_build_embree_binaries \
  && rm -rf dockerfile_build_embree_binaries embree
COPY dockerfile_build_ospray_binaries /dockerfile_build_ospray_binaries
COPY ospray_no_fast_math.patch /ospray_no_fast_math.patch
COPY ospray_ospcommon.patch /ospray_ospcommon.patch
RUN git clone --branch v1.8.5 --config advice.detachedHead=false --depth 1 --quiet https://github.com/ospray/ospray.git \
  && cd ospray \
  && git apply ../ospray_no_fast_math.patch ../ospray_ospcommon.patch \
  && cd .. \
  && rm -f ospray_no_fast_math.patch ospray_ospcommon.patch \
  && ./dockerfile_build_ospray_binaries \
  && rm -rf dockerfile_build_ospray_binaries ospray
COPY dockerfile_build_vtk_binaries /dockerfile_build_vtk_binaries
COPY vtk_rendering_ospray.patch /vtk_rendering_ospray.patch
RUN git clone --branch v8.2.0 --config advice.detachedHead=false --depth 1 --quiet https://gitlab.kitware.com/vtk/vtk.git \
  && cd vtk \
  && git apply ../vtk_rendering_ospray.patch \
  && cd .. \
  && rm -f vtk_rendering_ospray.patch \
  && ./dockerfile_build_vtk_binaries \
  && rm -rf dockerfile_build_vtk_binaries vtk \
  && sed -i 's|VTK_RUNTIME_DIRS "/opt/vtk/bin"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/bin"|g' /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake \
  && sed -i 's|VTK_RUNTIME_DIRS "/opt/vtk/lib"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/lib"|g' /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake \
  && sed -i 's|/opt/vtk|${_IMPORT_PREFIX}|g' /opt/vtk/lib/cmake/vtk-*/VTKTargets.cmake \
  && sed -i 's|${_IMPORT_PREFIX}/include/ospray;${_IMPORT_PREFIX}/include/ospray/SDK|${_IMPORT_PREFIX}/include/ospray|g' /opt/vtk/lib/cmake/vtk-*/VTKTargets.cmake \
  && sed -i 's|VTK_PYTHONPATH "/opt/vtk/lib/python3"|VTK_PYTHONPATH "${VTK_INSTALL_PREFIX}/lib/python3"|g' /opt/vtk/lib/cmake/vtk-*/Modules/vtkPython.cmake
RUN cd /opt/vtk \
  && tar -czf vtk-8.2.0-embree-3.5.2-ospray-1.8.5-python-$(python3 --version | sed -n -e 's/^.*\n*Python \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-qt-$(qmake --version | sed -n -e 's/^.*\n*Qt version \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-$(lsb_release --codename --short)-x86_64.tar.gz -- *
