# This Dockerfile and the accompanying shell script are used by the project
# maintainers to create the precompiled doxygen binaries that are downloaded
# during the build. They are neither called during the build nor expected to be
# called by most developers or users of the project.

ARG CODENAME=bionic
FROM ubuntu:$CODENAME
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
    bison \
    ca-certificates \
    cmake \
    flex \
    g++ \
    gcc \
    git \
    graphviz \
    libc6-dev \
    lsb-release \
    ninja-build \
    python3 \
  && rm -rf /var/lib/apt/lists/*
RUN git clone --branch Release_1_8_15 --config advice.detachedHead=false --depth 1 --quiet https://github.com/doxygen/doxygen.git \
  && mkdir -p doxygen-build /opt/doxygen \
  && cd doxygen-build \
  && cmake \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong  -Wno-unused-result -Wno-stringop-overflow' \
    -DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-unused-result -Wno-stringop-overflow' \
    -DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/doxygen \
    -DPYTHON_EXECUTABLE:PATH=/usr/bin/python3 \
    -GNinja \
    -Wno-dev \
    ../doxygen \
  && ninja install/strip \
  && cd .. \
  && cp doxygen/LICENSE /opt/doxygen/bin \
  && rm -rf doxygen doxygen-build
RUN cd /opt/doxygen/bin \
  && tar -czf doxygen-$(./doxygen --version)-$(lsb_release --codename --short)-x86_64.tar.gz doxygen LICENSE
