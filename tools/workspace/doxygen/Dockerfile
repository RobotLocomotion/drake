# This Dockerfile and the accompanying shell script are used by the project
# maintainers to create the precompiled doxygen binaries that are downloaded
# during the build. They are neither called during the build nor expected to be
# called by most developers or users of the project.

ARG UBUNTU_CODENAME=bionic
FROM ubuntu:${UBUNTU_CODENAME}
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet --quiet \
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
    wget \
  && rm -rf /var/lib/apt/lists/*
ARG DOXYGEN_VERSION=1.8.15
ARG DOXYGEN_SHA256=bd9c0ec462b6a9b5b41ede97bede5458e0d7bb40d4cfa27f6f622eb33c59245d
RUN wget --quiet "https://drake-mirror.csail.mit.edu/other/doxygen/doxygen-${DOXYGEN_VERSION}.src.tar.gz" \
  && echo "${DOXYGEN_SHA256}  doxygen-${DOXYGEN_VERSION}.src.tar.gz" | sha256sum --check - \
  && tar -xzf "doxygen-${DOXYGEN_VERSION}.src.tar.gz" \
  && mkdir -p doxygen-build /opt/doxygen \
  && cd doxygen-build \
  && cmake \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-return-local-addr -Wno-return-type -Wno-unused-result -Wno-stringop-overflow' \
    -DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-return-local-addr -Wno-return-type -Wno-unused-result -Wno-stringop-overflow' \
    -DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro' \
    -DCMAKE_INSTALL_PREFIX:PATH=/opt/doxygen \
    -DPYTHON_EXECUTABLE:PATH=/usr/bin/python3 \
    -GNinja \
    -Wno-dev \
    "../doxygen-${DOXYGEN_VERSION}" \
  && ninja -w dupbuild=warn install/strip \
  && cd .. \
  && cp -f "doxygen-${DOXYGEN_VERSION}/LICENSE" /opt/doxygen/bin \
  && rm -rf doxygen-*
RUN cd /opt/doxygen/bin \
  && tar -czf "doxygen-$(./doxygen --version)-$(lsb_release --codename --short)-$(uname --processor).tar.gz" doxygen LICENSE
