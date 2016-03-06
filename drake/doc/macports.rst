********
Mac (using macports)
********

We have provided some scripts to simplify the installation of prerequisites.  However, we do assume that you have already installed

* XQuartz / X11
* XCode and the command line tools (``xcode-select --install``)

Now you can install the prerequisites::

	sudo port selfupdate
	sudo port install cmake
	cd drake-distro
	make options # use the gui to choose which externals you want, then press c to configure, then g to generate makefiles and exit
	make download-all
	sudo ./install_prereqs.sh macports


Finally, add the line::

	export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/X11/lib/pkgconfig

to your ``.bash_profile`` or ``.bashrc``.

When you're done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
