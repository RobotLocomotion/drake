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
    brew tap homebrew/science
    brew tap robotlocomotion/director
    brew tap-pin cartr/qt4
    brew tap-pin robotlocomotion/director
    brew update
    brew upgrade
    brew install autoconf automake bazel boost clang-format cmake doxygen gcc \
      glib graphviz gtk+ jpeg libpng libtool libyaml mpfr ninja numpy \
      patchutils python \
      qt qt@4 qwt-qt4 scipy tinyxml vtk5 vtk@8.0 wget
    pip install -U beautifulsoup4 html5lib lxml PyYAML Sphinx

You may also want to install ``valgrind``, which can help debug memory issues in C++ code. Valgrind is available from homebrew with ``brew install valgrind``, but homebrew may encounter problems when installing ``valgrind`` on Mac OSX 10.12 (Sierra) or higher. See `this issue <https://github.com/Homebrew/homebrew-core/issues/4841#issuecomment-254217338>`_ for more details. If homebrew fails to install ``valgrind``, you can download it directly from `valgrind.org <http://valgrind.org/downloads/current.html>`_.

Add the line::

    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/X11/lib/pkgconfig

to your ``.bash_profile`` or ``.bashrc``.

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
