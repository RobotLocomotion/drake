*********************
OS X (using MacPorts)
*********************

We have provided some scripts to simplify the installation of prerequisites.  However, we do assume that you have already installed

* XQuartz / X11
* XCode and the command line tools (``xcode-select --install``)

You can install the prerequisites using the ``install_prereqs.sh`` script::

	sudo port selfupdate
	cd drake-distro
	sudo ./install_prereqs.sh macports
	make options  # use the GUI to choose which externals you want, then press 'c' to configure, then 'g' to generate makefiles and exit
	make download-all

Finally, add the line::

	export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/X11/lib/pkgconfig

to your ``.bash_profile`` or ``.bashrc``.

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
