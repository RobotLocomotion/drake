************************
Ubuntu 12.04 / 14.04 LTS
************************

You can install the prerequisites using the ``install_prereqs.sh`` script::

	sudo apt-get update
	sudo apt-get upgrade  # optional
	cd drake-distro
	sudo ./install_prereqs.sh ubuntu
	make options  # use the GUI to choose which externals you want, then press 'c' to configure, then 'g' to generate makefiles and exit
	make download-all

The version of the standard C++ libraries that are shipped with the Linux distribution of MATLAB is severely outdated and can cause problems when running mex files that are built against a newer version of the standard.  The typical error message in this case reports "Invalid MEX-Files"

To work around this issue, the symbolic link for the standard C++ library provided by MATLAB must be redirected to point to a more up-to-date version.

Update the symbolic link in MATLAB to point to the version that was installed earlier into ``/usr/lib``.  An example for MATLAB R2015b is shown below::

	cd /usr/local/MATLAB/R2015b/sys/os/glnxa64
	sudo rm libstdc++.so.6
	sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.9/libstdc++.so libstdc++.so.6

When you are done with these platform-specific steps, return to :doc:`from_source` to complete and test your installation.
