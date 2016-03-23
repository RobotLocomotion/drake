*********************
OS X (using MacPorts)
*********************

We assume that you have already installed:

* `Xcode <https://developer.apple.com/xcode/download/>`_ and the Command Line Tools (``xcode-select --install``)
* `XQuartz <http://www.xquartz.org/releases/>`_
* `Java SE Development Kit <http://www.oracle.com/technetwork/java/javase/downloads/>`_
* `MacPorts <https://www.macports.org/>_

Install the prerequisites::

    sudo port selfupdate
    sudo port install automake ccache cmake doxygen freeglut gcc49 glib2 \
      graphviz gtk2 jpeg libpng libtool mpfr mpich-default p5-html-form \
      p5-libwww-perl p5-term-readkey py-numpy py-pip python27 qt4-mac swig \
      valgrind wget
    sudo port install vtk5 +python27 +qt4_mac
    sudo port select --set mpich mpich-mp-fortran
    sudo pip install -U cpplint Sphinx

Add the line::

	export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/X11/lib/pkgconfig

to your ``.bash_profile`` or ``.bashrc``.

Download the external dependencies::

	cd drake-distro
	make options  # use the GUI to choose which externals you want, then press 'c' to configure, then 'g' to generate makefiles and exit
	make download-all

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
