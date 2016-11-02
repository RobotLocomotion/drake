*********************
OS X (using Homebrew)
*********************

We assume that you have already installed:

* `Xcode <https://developer.apple.com/xcode/download/>`_ and the Command Line Tools (``xcode-select --install``)
* `XQuartz <http://www.xquartz.org/releases/>`_
* `Java SE Development Kit <http://www.oracle.com/technetwork/java/javase/downloads/>`_
* `Homebrew <http://brew.sh/>`_

Install the prerequisites::

    brew tap homebrew/python
    brew tap homebrew/science
    brew tap robotlocomotion/director
    brew update
    brew upgrade
    brew install autoconf automake cmake doxygen gcc glib graphviz gtk+ jpeg \
      libpng libtool libyaml mpfr ninja numpy python qt qwt valgrind wget
    brew install vtk5 --with-qt
    pip install -U beautifulsoup4 html5lib Sphinx PyYAML

Add the line::

    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/X11/lib/pkgconfig

to your ``.bash_profile`` or ``.bashrc``.

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
