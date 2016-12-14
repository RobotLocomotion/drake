.. _code-style-tools:

*******************************
Tools for Code Style Compliance
*******************************

This section provides a list of tools that some have found useful for ensuring their
code abides by :ref:`Drake's coding style <code-style-guide>`. The list is by no means comprehensive.
If your favorite tools or methodologies are not listed, we would be delighted to learn about them. Please
document your trick and submit a pull request!

.. contents:: `Table of contents`
   :depth: 3
   :local:

C/C++
=====

.. _code-style-tools-clang-format:

Clang-Format
------------

Installation
^^^^^^^^^^^^

On Ubuntu, first determine which versions are available::

    apt-cache search clang-format

Then install the version you prefer::

    sudo apt-get install clang-format-[version]

Once installed, create a symbolic link so you don't need to type the version number every time you execute it::

    sudo ln -s /usr/bin/clang-format-[version] /usr/local/bin/clang-format

On OSX::

    brew install clang-format

You can check whether you've installed it correctly by executing::

    clang-format --help

Usage
^^^^^^^^^

To run clang-format::

    clang-format -i -style=file [file name]

cpplint
-------

Usage
^^^^^

`cpplint <https://github.com/google/styleguide/tree/gh-pages/cpplint>`_
is a tool for finding compliance violations. Here is the command::

    drake-distro/drake/common/test/cpplint_wrapper.py

By default, all files in Drake are checked using the default settings.
Consult the program's `--help` for more detailed options.
