**********
Ubuntu (using apt-get)
**********

You can install the prerequisites::

	sudo apt-get update
	sudo apt-get install cmake
	sudo apt-get install cmake-curses-gui
	sudo apt-get install build-essential
	cd drake-distro
	make options # use the gui to choose which externals you want, then press c to configure, then g to generate makefiles and exit
	make download-all
	sudo ./install_prereqs.sh ubuntu

The version of the standard C++ libraries that are shipped with the Linux distribution of MATLAB is severely outdated and can cause problems when running mex files that are built against a newer version of the standard.  The typical error message in this case reports "Invalid MEX-Files"

To work around this issue, the symbolic link for the standard C++ library provided by MATLAB must be redirected to point to a more up-to-date version.

First, make sure that a suitable version of the standard library is installed::

	sudo apt-get install g++-4.4

Now, the symbolic link in MATLAB must be updated to point to the version that was just installed in ``/usr/lib``.  An example for MATLAB R2014a is shown below::

	cd /usr/local/MATLAB/R2014a/sys/os/glnxa64
	sudo rm libstdc++.so.6
	sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.4/libstdc++.so libstdc++.so.6

When you're done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
