***************
OS X El Capitan
***************

We assume that you have already installed:

* `Xcode 7.3.1 or above <https://developer.apple.com/xcode/download/>`_ and the Command Line Tools (``xcode-select --install``)
* `Java SE Development Kit 8 or above <http://www.oracle.com/technetwork/java/javase/downloads/>`_

Using the Bazel Build System
============================

Prerequisite setup is automated. Simply run::

    ./setup/mac/install_prereqs.sh

After running the script, return to :doc:`from_source` to complete and test your
installation.

Using the Legacy CMake Build System
===================================

Install Homebrew::

    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

Install the prerequisites::

    brew update
    brew upgrade
    brew install \
       autoconf \
       automake \
       boost \
       clang-format \
       cmake \
       doxygen \
       gcc \
       glib \
       libpng \
       libtool \
       ninja \
       numpy \
       patchutils \
       pkg-config \
       python \
       tinyxml \
       valgrind \
       zlib
    pip2 install --upgrade pip Sphinx

If you will be building Director, some additional prerequisites may be installed
as follows::

    brew tap cartr/qt4
    brew tap homebrew/science
    brew tap robotlocomotion/director
    brew install libyaml qt@4 scipy vtk5
    pip2 install --upgrade lxml PyYAML

When you are done with these platform-specific steps, return to
:doc:`from_source` to complete and test your installation.
