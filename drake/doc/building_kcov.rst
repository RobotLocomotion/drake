.. _building-kcov:

*****************************
Building ``kcov`` From Source
*****************************

As of this writing, the recommended way to get ``kcov`` is to build from
upstream sources from GitHub. Ubuntu Xenial (for example) has packaged
versions, but they have known problems not evident when using upstream.

Ubuntu-only Preparation
=======================

On Ubuntu, install the build dependencies::

  sudo apt install binutils-dev libdw-dev libelf-dev libiberty-dev

OSX and Ubuntu Build Steps
==========================

To build from source, follow these steps::

  git clone git@github.com:SimonKagstrom/kcov.git
  cd kcov
  mkdir build
  cd build
  cmake ..
  cmake --build .

The resulting binary will be in ``kcov/build/src/kcov``. Create a symbolic link
to the binary from some directory on your ``$PATH``.

OSX Run-time Requirements
=========================

On OSX, be sure that your account has developer mode enabled, which gives you
the privileges necessary to run debuggers and similar tools. If you are an
administrator, use this command::

  sudo /usr/sbin/DevToolsSecurity --enable

