*************************************************
Centos / Red Hat Enterprise Linux 7.2 / Fedora 23
*************************************************

You can install the prerequisites using the ``install_prereqs.sh`` script::

	sudo yum update  # optional
	cd drake-distro
	sudo ./install_prereqs.sh fedora
	make options  # use the GUI to choose which externals you want, then press 'c' to configure, then 'g' to generate makefiles and exit
	make download-all
