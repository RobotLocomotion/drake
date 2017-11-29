.. _build_from_source:

***********************************
Source installation (macOS, Ubuntu)
***********************************

.. _getting_drake:

Getting Drake
=============

We recommend that you `setup SSH access to Github.com <https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/>`_
to avoid needing to type your password each time you access it. The following
instructions assume you have uploaded your public SSH key to your Github
account.

Now run::

    git clone git@github.com:RobotLocomotion/drake.git drake-distro


Note: the build process may encounter problems if you have unusual characters
like parentheses in the absolute path to the drake-distro directory
(see `#394 <https://github.com/RobotLocomotion/drake/issues/394>`_).

The above ``git clone`` command will configure Drake's primary repository as a
remote called ``origin``. We recommend that you configure your fork of Drake's
primary repository as the ``origin`` remote and Drake's primary repository as
the ``upstream`` remote. This can be done by executing the following commands::

    cd drake-distro
    git remote set-url origin git@github.com:[your github user name]/drake.git
    git remote add upstream git@github.com:RobotLocomotion/drake.git
    git remote set-url --push upstream no_push

.. _platform_specific_setup:

Mandatory platform specific instructions
========================================

Before running the build, you must follow some one-time platform-specific
setup steps:

.. toctree::
    :maxdepth: 1

    mac
    ubuntu_xenial

See :ref:`supported configurations <supported-configurations>`
for the configurations and platforms that Drake officially supports.
All else being equal, we would recommend developers use Ubuntu Xenial.

.. _build_with_bazel:

Build with Bazel
================

For instructions, jump to :ref:`Using Bazel <using_bazel>`, or check out the
full details at:

.. toctree::
    :maxdepth: 1

    bazel
