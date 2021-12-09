# This Dockerfile and the accompanying shell script are used by the project
# maintainers to create the precompiled vtk binaries that are downloaded during
# the build. They are neither called during the build nor expected to be called
# by most developers or users of the project.

ARG UBUNTU_CODENAME=bionic
FROM ubuntu:${UBUNTU_CODENAME}
# NOTE: a newer CMake on bionic is needed, gpg and wget get installed to obtain
# the newer CMake (see: https://apt.kitware.com/). They can be removed when
# bionic support is dropped.
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
    ca-certificates \
    g++ \
    gcc \
    git \
    gpg \
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
    libqt5opengl5-dev \
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
    wget \
    zlib1g-dev \
  && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
  | gpg --dearmor - \
  | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
  && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" \
  | tee /etc/apt/sources.list.d/kitware.list >/dev/null \
  && apt-get update \
  && rm /usr/share/keyrings/kitware-archive-keyring.gpg \
  && apt-get install --no-install-recommends --quiet --yes kitware-archive-keyring \
  && apt-get install --no-install-recommends --quiet --yes \
    "cmake-data=3.16.3-*" "cmake=3.16.3-*" \
  && cmake --version \
  && rm -rf /var/lib/apt/lists/*
RUN git clone --branch v9.1.0 --config advice.detachedHead=false --depth 1 --quiet \
    https://gitlab.kitware.com/vtk/vtk.git /vtk
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
    -DPython3_EXECUTABLE:PATH=/usr/bin/python3 \
    -DVTK_PYTHON_VERSION:STRING=3 \
    -DVTK_WRAP_PYTHON:BOOL=ON \
    -DVTK_GROUP_ENABLE_Qt:STRING=YES \
    -DVTK_QT_VERSION:STRING=5 \
    -DVTK_MODULE_ENABLE_VTK_GUISupportQtQuick:STRING=DONT_WANT \
    -DVTK_LEGACY_REMOVE:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_doubleconversion:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_eigen:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_expat:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_gl2ps:BOOL=OFF \
    -DVTK_MODULE_USE_EXTERNAL_VTK_freetype:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_glew:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_hdf5:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_jpeg:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_jsoncpp:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_libxml2:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_lz4:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_lzma:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_netcdf:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_ogg:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_png:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_pugixml:BOOL=OFF \
    -DVTK_MODULE_USE_EXTERNAL_VTK_sqlite:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_theora:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_tiff:BOOL=ON \
    -DVTK_MODULE_USE_EXTERNAL_VTK_utf8:BOOL=OFF \
    -DVTK_MODULE_USE_EXTERNAL_VTK_zlib:BOOL=ON \
    -GNinja \
    -Wno-dev \
    /vtk \
  && ninja install/strip \
  && rm -rf /vtk /vtk-build \
  && find /opt/vtk -name __pycache__ -type d -exec rm -rf {} +
RUN cd /opt/vtk \
  && tar --create --gzip --file vtk-9.1.0-1-python-$(python3 --version | sed --quiet --expression 's/^.*\n*Python \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-qt-$(qmake --version | sed --quiet --expression 's/^.*\n*Qt version \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-$(lsb_release --codename --short)-x86_64.tar.gz -- *
