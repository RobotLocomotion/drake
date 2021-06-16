# This Dockerfile and the accompanying shell script are used by the project
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
    libdouble-conversion-dev \
    libeigen3-dev \
    libexpat1-dev \
    libfreetype6-dev \
    libgl1-mesa-dev \
    libglew-dev \
    libglib2.0-dev \
    libglu1-mesa-dev \
    libglvnd-dev \
    libhdf5-dev \
    libjpeg-turbo8-dev \
    libjsoncpp-dev \
    liblz4-dev \
    liblzma-dev \
    libnetcdf-dev \
    libogg-dev \
    libpng-dev \
    libqt5x11extras5-dev \
    libsqlite3-dev \
    libtheora-dev \
    libtiff5-dev \
    libxml2-dev \
    libxt-dev \
    lsb-release \
    ninja-build \
    python3-dev \
    qt5-default \
    qttools5-dev \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*
COPY *.patch /vtk-patch/
RUN git clone --branch v8.2.0 --config advice.detachedHead=false --depth 1 --quiet \
    https://gitlab.kitware.com/vtk/vtk.git /vtk \
  && git -C /vtk apply /vtk-patch/*.patch \
  && rm -rf /vtk-patch
RUN mkdir -p /vtk-build /opt/vtk \
  && cd /vtk-build \
  && cmake \
    -DBUILD_TESTING:BOOL=OFF \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations' \
    -DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations' \
    -DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/vtk \
    -DCMAKE_MODULE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_SHARED_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DPYTHON_EXECUTABLE:PATH=/usr/bin/python3 \
    -DVTK_ENABLE_VTKPYTHON:BOOL=OFF \
    -DVTK_Group_Qt:BOOL=ON \
    -DVTK_LEGACY_REMOVE:BOOL=ON \
    -DVTK_QT_VERSION:STRING=5 \
    -DVTK_USE_SYSTEM_DOUBLECONVERSION:BOOL=ON \
    -DVTK_USE_SYSTEM_EIGEN:BOOL=ON \
    -DVTK_USE_SYSTEM_EXPAT:BOOL=ON \
    -DVTK_USE_SYSTEM_FREETYPE:BOOL=ON \
    -DVTK_USE_SYSTEM_GLEW:BOOL=ON \
    -DVTK_USE_SYSTEM_HDF5:BOOL=ON \
    -DVTK_USE_SYSTEM_JPEG:BOOL=ON \
    -DVTK_USE_SYSTEM_JSONCPP:BOOL=ON \
    -DVTK_USE_SYSTEM_LIBXML2:BOOL=ON \
    -DVTK_USE_SYSTEM_LZ4:BOOL=ON \
    -DVTK_USE_SYSTEM_LZMA:BOOL=ON \
    -DVTK_USE_SYSTEM_NETCDF:BOOL=ON \
    -DVTK_USE_SYSTEM_OGG:BOOL=ON \
    -DVTK_USE_SYSTEM_PNG:BOOL=ON \
    -DVTK_USE_SYSTEM_SQLITE:BOOL=ON \
    -DVTK_USE_SYSTEM_THEORA:BOOL=ON \
    -DVTK_USE_SYSTEM_TIFF:BOOL=ON \
    -DVTK_USE_SYSTEM_ZLIB:BOOL=ON \
    -DVTK_WRAP_PYTHON:BOOL=ON \
    -GNinja \
    -Wno-dev \
    /vtk \
  && ninja install/strip \
  && rm -rf /vtk /vtk-build \
  && find /opt/vtk -name __pycache__ -type d -exec rm -rf {} + \
  && sed --in-place \
    's|VTK_RUNTIME_DIRS "/opt/vtk/bin"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/bin"|g' \
    /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake \
  && sed --in-place \
    's|VTK_RUNTIME_DIRS "/opt/vtk/lib"|VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/lib"|g' \
    /opt/vtk/lib/cmake/vtk-*/VTKConfig.cmake \
  && sed --in-place \
    's|/opt/vtk|${_IMPORT_PREFIX}|g' \
    /opt/vtk/lib/cmake/vtk-*/VTKTargets.cmake \
  && sed --in-place \
    's|VTK_PYTHONPATH "/opt/vtk/lib/python3"|VTK_PYTHONPATH "${VTK_INSTALL_PREFIX}/lib/python3"|g' \
    /opt/vtk/lib/cmake/vtk-*/Modules/vtkPython.cmake
RUN cd /opt/vtk \
  && tar --create --gzip --file vtk-8.2.0-1-python-$(python3 --version | sed --quiet --expression 's/^.*\n*Python \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-qt-$(qmake --version | sed --quiet --expression 's/^.*\n*Qt version \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-$(lsb_release --codename --short)-x86_64.tar.gz -- *
