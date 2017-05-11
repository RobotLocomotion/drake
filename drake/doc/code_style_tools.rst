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

All Languages
=============

.. _code-style-tools-all-languages:

When using the Bazel build system, code style tests are run by default during
``bazel test`` and its results are cached so that only edited files are
re-checked.
In other words, no special action is required to use the tool.

However, you may still invoke code style checks directly if desired, as
follows::

  cd /path/to/drake-distro
  bazel test --config lint ...                  # Only run style checks; don't build or test anything else.
  bazel test --config lint //drake/common/...   # Check common/ and its child subdirectories.

  cd drake/systems/framework
  bazel test --config lint ...                  # Check systems/framework/ and its child subdirectories.

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
^^^^^

To run clang-format::

    clang-format -i -style=file [file name]

cpplint
-------

`cpplint <https://github.com/google/styleguide/tree/gh-pages/cpplint>`_
is a tool for finding compliance violations.

Usage via Bazel
^^^^^^^^^^^^^^^

Use the command in `All Languages`_, but change the configuration to use
``cpplint``::

  bazel test --config cpplint ...

Usage without Bazel
^^^^^^^^^^^^^^^^^^^

Here is the command::

  drake-distro/drake/common/test/cpplint_wrapper.py

By default, all files in Drake are checked using the default settings.
Consult the program's `--help` for more detailed options.


Python
======

.. _code-style-tools-pycodestyle:

pycodestyle.py
--------------

``pycodestyle``, which used to be ``pep8``, is the primary code style checker
used within Drake.

Usage via Bazel
^^^^^^^^^^^^^^^

Use the command in `All Languages`_, but change the configuration to use
``pycodestyle``::

  bazel test --config pycodestyle ...

Usage without Bazel
^^^^^^^^^^^^^^^^^^^

First, ensure that you have ``pycodestyle`` installed. On Ubuntu, install via
``pip``::

  pip install --user -U pycodestyle

To your ``.bashrc`` or ``.profile``, add the line::

  export PATH=$HOME/.local/bin:$PATH

On OSX, similarly::

  pip install --user -U pycodestyle

To your ``.bashrc`` or ``.bash_profile``, add the line::

  export PATH=$HOME/Library/Python/2.7/bin:$PATH

Run ``pycodestyle`` like this::

  pycodestyle [file name]

It is possible to suppress pycodestyle errors, either via configuration files
in various locations, or command line options. However, it does not support
individual suppressions via source code comments. Consult the program's
`--help` or ``pycodestyle.readthedocs.org`` for details.

.. _code-style-tools-pylint:

pylint
------

Installation
^^^^^^^^^^^^

Like ``pycodestyle``, ``pylint`` should be installed via ``pip``.
Instructions are exactly analogous to those for ``pycodestyle`` above,
substituting the package name
``pylint``.

Usage
^^^^^

To run ``pylint``, with some noisy reports and questionable rules suppressed::

  pylint --reports=n --disable=C,R [file name]

It is possible to suppress pylint complaints, either via configuration files in
various locations, or by special comments in python source files. Consult the
program's `--help` or ``docs.pylint.org`` for details.
