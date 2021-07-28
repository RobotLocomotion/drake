# This Dockerfile and the accompanying shell script and patch files are used by
# the project maintainers to create the precompiled drake-visualizer binaries
# that are downloaded during the build. They are neither called during the
# build nor expected to be called by most developers or users of the project.

# The vtk tar.gz archive for ${UBUNTU_CODENAME} must have been copied or
# downloaded to same directory as this Dockerfile before calling docker build.

ARG UBUNTU_CODENAME=bionic
FROM ubuntu:${UBUNTU_CODENAME}
# TODO(jamiesnape): Decoupling this from the VTK build has lead to an explosion
# of non-obvious dependencies. We need to re-think directory structure and
# share common instructions if we keep building our own VTK.
#
# Note that we often use a development package even though we only need a
# runtime package since the development package names are usually the same in
# multiple Ubuntu distributions.
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
    ca-certificates \
    chrpath \
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
    unzip \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*
# Note that we need a recent CMake to ensure we get the updated FindOpenGL and
# therefore link the correct flavor of OpenGL library (GLVND). With the version
# of CMake from the Ubuntu distribution, we would need even more drastic
# patching of drake-visualizer.
RUN export APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 \
  && apt-key --keyring /etc/apt/trusted.gpg.d/bazel.gpg adv --quiet \
    --fetch-keys https://bazel.build/bazel-release.pub.gpg \
  && apt-key --keyring /etc/apt/trusted.gpg.d/kitware.gpg adv --quiet \
    --fetch-keys https://apt.kitware.com/keys/kitware-archive-latest.asc \
  && echo 'deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8' \
    > /etc/apt/sources.list.d/bazel.list \
  && echo "deb https://apt.kitware.com/ubuntu $(lsb_release --codename --short) main" \
    > /etc/apt/sources.list.d/kitware.list \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
    bazel \
    cmake \
  && rm -rf /var/lib/apt/lists/*
# Note that we symlink /usr/bin/python since the package names to install an
# executable named as such are not the same in different Ubuntu packages. The
# actual version of Python does not matter, either 2 or 3 would fix Bazel.
#
# Note that we use LCM and its dependency JChart2D from Drake, partly due to
# mangled name of the LCM library in Drake. We do not use JChart2D, but
# "find_package(LCM)" would fail without it. LCM is LGPL 2.1 so we
# should not link it statically like we do for the other C/C++ dependencies of
# drake-visualizer.
#
# TODO(jamiesnape): Maybe we should bundle a separate version anyway, but are
# different LCM versions always compatible with Drake?
#
# Note that the added options are to follow the Debian build hardening
# guidelines, which is good practice anyway.
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
    @lcm//:install -- /opt/drake 2>&1 \
  && bazel run \
    --copt=-fstack-protector-strong \
    --host_copt=-fstack-protector-strong \
    --linkopt=-Wl,-Bsymbolic-functions \
    --linkopt=-Wl,-z,now \
    --linkopt=-Wl,-z,relro \
    //tools/workspace/net_sf_jchart2d:install -- /opt/drake 2>&1 \
  && bazel clean --expunge 2>&1 \
  && bazel shutdown \
  && rm -rf /drake /root/.cache /usr/bin/python
# Copy the prebuilt VTK binary from the build context to the container.
#
# TODO(jamiesnape): Maybe we should bundle a version of VTK or use a system
# version, both of which would be huge in size, regardless.
COPY vtk-*.tar.gz /tmp/
RUN tar --extract --gzip --directory /opt/drake \
    --file /tmp/vtk-*-$(lsb_release --codename --short)-*.tar.gz \
  && rm -rf /tmp/vtk-*.tar.gz
# Copy any patches to drake-visualizer from the build context to the container.
COPY *.patch /director-patch/
RUN git clone --branch master --config advice.detachedHead=false --quiet \
    --single-branch https://github.com/RobotLocomotion/director.git /director \
  && git -C /director checkout 4c3e570a6797ff840c74067c742455daaa113d93 \
  && git -C /director apply /director-patch/*.patch \
  && rm -rf /director-patch
# Note that some flags are added to "CMAKE_CXX_FLAGS", etc., rather than their
# usual variables (e.g., "CMAKE_CXX_STANDARD") since drake-visualizer does not
# pass those flags down to its external project builds.
#
# Note that most added flags are to follow the Debian build hardening
# guidelines, which is good practice anyway.
#
# TODO(jamiesnape): The flag "-isystem/opt/drake/include/lcm" should not be
# necessary. Something is wrong somewhere.
#
# Note that we remove several installed components that are either broken,
# unused by Drake, or duplicated by Drake. Exactly which are broken vs. unused
# vs. duplicated has been lost in time, may be wrong now.
#
# TODO(jamiesnape): Maybe we should bundle some of the duplicate components
# anyway.
#
# Note that we have to manually call "strip" since there is no "install/strip"
# target when building drake-visualizer and its dependencies as external
# projects.
#
# TODO(jamiesnape): Honestly, the RPATHs should not need separate patching, but
# CMake has been changed substantially since drake-visualizer was written and
# it handles things differently and more correctly now, thankfully.
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
    -Wno-deprecated \
    -Wno-dev \
    -S /director/distro/superbuild \
    -B /director-build \
  && export PYTHONWARNINGS=ignore::SyntaxWarning \
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
# Create archive named dv-<version>-python-<python version>-qt-<qt version>
#   -vtk-<vtk version>-<distribution codename>-<processor architecture>.tar.gz,
RUN cd /opt/director \
  && tar --create --gzip --file dv-0.1.0-406-g4c3e570a-python-$(python3 --version 2>&1 | sed --quiet --expression 's/^.*\n*Python \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-qt-$(qmake --version | sed --quiet --expression 's/^.*\n*Qt version \([0-9]\+\)\.\([0-9]\+\)\.\([0-9]\+\).*/\1.\2.\3/p')-vtk-8.2.0-$(lsb_release --codename --short)-$(uname --processor).tar.gz -- *
