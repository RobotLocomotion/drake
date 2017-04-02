***************
OS X El Capitan
***************

We assume that you have already installed:

* `Xcode 7.3.1 or above <https://developer.apple.com/xcode/download/>`_ and the Command Line Tools (``xcode-select --install``)
* `XQuartz 2.7.8 or above <https://www.xquartz.org/releases/>`_
* `Java SE Development Kit 7 or above <http://www.oracle.com/technetwork/java/javase/downloads/>`_
* `Homebrew <http://brew.sh/>`_

Install the prerequisites::

    brew tap cartr/qt4
    brew tap homebrew/python
    brew tap homebrew/science
    brew tap robotlocomotion/director
    brew tap-pin cartr/qt4
    brew tap-pin robotlocomotion/director
    brew update
    brew upgrade
    brew install autoconf automake bazel clang-format cmake doxygen gcc glib \
      graphviz gtk+ jpeg libpng libtool libyaml mpfr ninja numpy python \
      qt qwt scipy valgrind vtk5 wget
    pip install -U beautifulsoup4 html5lib lxml PyYAML Sphinx

Add the line::

    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/X11/lib/pkgconfig

to your ``.bash_profile`` or ``.bashrc``.

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
