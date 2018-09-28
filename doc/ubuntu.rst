.. _build_from_source_ubuntu:

******
Ubuntu
******

Prerequisite setup is automated. For Ubuntu Xenial, simply run::

    sudo ./setup/ubuntu/16.04/install_prereqs.sh

For Ubuntu Bionic, simply run::

    sudo ./setup/ubuntu/18.04/install_prereqs.sh

You may need to respond to interactive prompts to confirm that you agree to add
the appropriate Drake `apt` repository for your release
(``https://drake-apt.csail.mit.edu/xenial/`` or
``https://drake-apt.csail.mit.edu/bionic/``) to your system for certain
development packages.
