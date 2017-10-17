.. _build_from_source_xenial:

*************************
Ubuntu 16.04 LTS (Xenial)
*************************

The following instructions are written for Ubuntu 16.04 LTS, which is a
supported Drake platform.

Prerequisite setup on Ubuntu 16.04 is automated. Simply run::

    sudo ./setup/ubuntu/16.04/install_prereqs.sh

You may need to respond to interactive prompts to confirm that you agree to add
various `apt` repositories to your system.

Using the Legacy CMake Build System
===================================

To use the legacy CMake build, you must also complete the following step::

    sudo apt install --no-install-recommends cmake cmake-curses-gui make
