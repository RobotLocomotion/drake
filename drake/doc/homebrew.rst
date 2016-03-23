********
Mac (using homebrew -- recommended for mac)
********

We have provided some scripts to simplify the installation of prerequisites.  However, we do assume that you have already installed

* `XQuartz / X11 <http://www.xquartz.org>`_
* XCode and the command line tools (``xcode-select --install``)
* `Java Development Kit (JDK) <http://www.oracle.com/technetwork/java/javase/downloads/index.html>`_

Now you can install the prerequistes::

	brew update
	brew install cmake
	cd drake-distro
	make options # use the gui to choose which externals you want, then press c to configure, then g to generate makefiles and exit
	make download-all
	./install_prereqs.sh homebrew


Finally, add the line::

	export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/X11/lib/pkgconfig

to your ``.bash_profile`` or ``.bashrc``.


When you're done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.