# This Dockerfile and the accompanying shell script are used by the project
# maintainers to create the precompiled drake-visualizer binaries that are
# downloaded during the build. They are neither called during the build nor
# expected to be called by most developers or users of the project.

# The vtk tar.gz archive for ${UBUNTU_CODENAME} must have been copied or
# downloaded to same directory as this Dockerfile before calling docker build.

ARG UBUNTU_CODENAME=bionic
FROM ubuntu:${UBUNTU_CODENAME}
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
    ca-certificates \
    chrpath \
    cmake \
    default-jdk \
    file \
    g++ \
    gcc \
    git \
    gnupg \
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
    libjpeg8-dev \
    libjsoncpp-dev \
    liblz4-dev \
    liblzma-dev \
    libnetcdf-dev \
    libogg-dev \
    libpng-dev \
    libqt5opengl5-dev \
    libqt5x11extras5-dev \
    libsqlite3-dev \
    libtbb-dev \
    libtheora-dev \
    libtiff5-dev \
    libutfcpp-dev \
    libxml2-dev \
    libxt-dev \
    lsb-release \
    ninja-build \
    patchelf \
    python3-dev \
    python3-lxml \
    python3-numpy \
    python3-scipy \
    python3-yaml \
    qt5-default \
    qtbase5-private-dev \
    qtmultimedia5-dev \
    qttools5-dev \
    zlib1g-dev \
  && export APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 \
  && apt-key adv --fetch-keys \
    https://bazel.build/bazel-release.pub.gpg \
    https://apt.kitware.com/keys/kitware-archive-latest.asc \
  && echo 'deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8' \
    > /etc/apt/sources.list.d/bazel.list \
  && echo "deb https://apt.kitware.com/ubuntu $(lsb_release --codename --short) main" \
    > /etc/apt/sources.list.d/kitware.list \
  && apt-get update --quiet --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
    bazel \
    cmake \
  && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --quiet https://github.com/RobotLocomotion/drake.git \
    /drake \
  && ln /usr/bin/python3 /usr/bin/python \
  && cd /drake \
  && bazel run \
    --copt=-fstack-protector-strong \
    --host_copt=-fstack-protector-strong \
    --linkopt=-Wl,-Bsymbolic-functions \
    --linkopt=-Wl,-z,now \
    --linkopt=-Wl,-z,relro \
    @lcm//:install -- /opt/drake \
  && bazel run \
    --copt=-fstack-protector-strong \
    --host_copt=-fstack-protector-strong \
    --linkopt=-Wl,-Bsymbolic-functions \
    --linkopt=-Wl,-z,now \
    --linkopt=-Wl,-z,relro \
    //tools/workspace/net_sf_jchart2d:install -- /opt/drake \
  && bazel clean --expunge \
  && bazel shutdown \
  && rm -rf /drake /root/.cache /usr/bin/python
COPY vtk-*.tar.gz /tmp/
RUN tar --extract --gzip --directory /opt/drake \
    --file /tmp/vtk-*-$(lsb_release --codename --short)-*.tar.gz \
  && rm -rf /tmp/vtk-*.tar.gz
COPY *.patch /director-patch/
RUN git clone --branch master --config advice.detachedHead=false --quiet \
    --single-branch https://github.com/RobotLocomotion/director.git /director \
  && git -C /director checkout 4c3e570a6797ff840c74067c742455daaa113d93 \
  && git -C /director apply /director-patch/*.patch \
  && rm -rf /director-patch
RUN mkdir -p /director-build /opt/director \
  && cmake \
    -DBUILD_SHARED_LIBS:BOOL=OFF \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations -Wno-format-security' \
    -DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -isystem/opt/drake/include/lcm -Wno-conversion-null -Wno-deprecated-declarations -Wno-format-security -std=c++17' \
    -DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,--as-needed -Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/director \
    -DCMAKE_MODULE_LINKER_FLAGS:STRING='-Wl,--as-needed -Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_PREFIX_PATH:PATH=/opt/drake \
    -DCMAKE_SHARED_LINKER_FLAGS:STRING='-Wl,--as-needed -Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DDD_QT_VERSION:STRING=5 \
    -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
    -DUSE_EXTERNAL_INSTALL:BOOL=ON \
    -DUSE_LCM:BOOL=ON \
    -DUSE_LCMGL:BOOL=OFF \
    -DUSE_LIBBOT:BOOL=OFF \
    -DUSE_SYSTEM_EIGEN:BOOL=ON \
    -DUSE_SYSTEM_LCM:BOOL=ON \
    -DUSE_SYSTEM_LIBBOT:BOOL=OFF \
    -DUSE_SYSTEM_VTK:BOOL=ON \
    -DVTK_DIR:PATH=/opt/drake/lib/cmake/vtk-8.2 \
    -GNinja \
    -Wno-dev \
    -S /director/distro/superbuild \
    -B /director-build \
  && ninja -C /director-build \
  && rm -rf \
    /director \
    /director-build \
    /opt/director/bin/directorPython \
    /opt/director/include \
    /opt/director/lib/*.a \
    /opt/director/lib/bot2-core \
    /opt/director/lib/cmake \
    /opt/director/lib/pkgconfig \
    /opt/director/lib/python3 \
    /opt/director/lib/robotlocomotion-lcmtypes \
  && find /opt/director -name __pycache__ -type d -exec rm -rf {} + \
  && strip \
    /opt/director/bin/drake-visualizer \
    /opt/director/lib/*.so \
    /opt/director/lib/python3.*/site-packages/director/*.so \
    /opt/director/lib/python3.*/site-packages/director/thirdparty/*.so \
  && chrpath --replace '$ORIGIN/../lib' /opt/director/bin/drake-visualizer \
  && chrpath --replace '$ORIGIN' /opt/director/lib/libddApp.so \
  && chrpath --replace '$ORIGIN/../../..' \
    /opt/director/lib/python3.*/site-packages/director/vtkDRCFiltersPython.so \
  && chrpath --delete \
    /opt/director/lib/python3.*/site-packages/director/thirdparty/*.so
RUN cd /opt/director \
  && tar --create --gzip --file dv-0.1.0-406-g4c3e570a-python-$(python3 --version 2>&1 | sed --quiet --expression 's/^.*\n*Python \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-qt-$(qmake --version | sed --quiet --expression 's/^.*\n*Qt version \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-vtk-8.2.0-$(lsb_release --codename --short)-$(uname --processor).tar.gz -- *
