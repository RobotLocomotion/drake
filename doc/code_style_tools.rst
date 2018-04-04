.. _code-style-tools:

*******************************
Tools for Code Style Compliance
*******************************

This section provides a list of tools that some have found useful for ensuring
their code abides by :ref:`Drake's coding style <code-style-guide>`. The list
is by no means comprehensive.
If your favorite tools or methodologies are not listed, we would be delighted
to learn about them. Please document your trick and submit a pull request!

.. contents:: `Table of contents`
   :depth: 3
   :local:

Automated style checks
======================

Code style tests are run by default during ``bazel test`` and the results are
cached so that only edited files are re-checked.  In other words, no special
action is required by a developer.

However, you may still invoke code style checks directly if desired, as
follows::

  cd /path/to/drake
  bazel test --config lint //...         # Only run style checks; don't build or test anything else.
  bazel test --config lint //common/...  # Check common/ and its child subdirectories.

User manuals for the style-checking tools are as follows:

- C/C++: See the cpplint ``USAGE`` string at
  https://github.com/google/styleguide/blob/gh-pages/cpplint/cpplint.py.

  - In particular, note the ``// NOLINT(foo/bar)`` syntax to disable a warning.

- Python: See the pycodestyle manual at
  http://pycodestyle.readthedocs.io/en/latest/intro.html.

  - The syntax ``# noqa`` can be used to quiet the warning about an overly-long
    line.

- Bazel: Uses both pycodestyle like Python, and also :ref:`buildifier <buildifier>`.


Manual style fixups
===================

.. _code-style-tools-clang-format:

C/C++: Clang-Format
-------------------

The
:ref:`Mandatory platform specific instructions <platform_specific_setup>`
already install ``clang-format-4.0``.

You can check whether you've installed it correctly by executing::

    clang-format-4.0 --help

To run clang-format::

    clang-format-4.0 -i -style=file [file name]
