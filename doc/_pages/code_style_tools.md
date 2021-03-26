---
title: Tools for Code Style Compliance
---

This section provides a list of tools that some have found useful for ensuring
their code abides by [Drake's coding style](/code_style_guide.html). The list
is by no means comprehensive.
If your favorite tools or methodologies are not listed, we would be delighted
to learn about them. Please document your trick and submit a pull request!


# Automated style checks

Code style tests are run by default during ``bazel test`` and the results are
cached so that only edited files are re-checked.  In other words, no special
action is required by a developer.

However, you may still invoke code style checks directly if desired, as
follows:

```
cd /path/to/drake
bazel test --config lint //...         # Only run style checks; don't build or test anything else.
bazel test --config lint //common/...  # Check common/ and its child subdirectories.
```

User manuals for the style-checking tools are as follows:

* C/C++: See the cpplint ``USAGE`` string at
  [https://github.com/google/styleguide/blob/gh-pages/cpplint/cpplint.py](https://github.com/google/styleguide/blob/gh-pages/cpplint/cpplint.py).
  * In particular, note the ``// NOLINT(foo/bar)`` syntax to disable a warning.
* Python: See the pycodestyle manual at
  [http://pycodestyle.readthedocs.io/en/latest/intro.html](http://pycodestyle.readthedocs.io/en/latest/intro.html).
  * The syntax ``# noqa`` can be used to quiet the warning about an overly-long
    line.
* Bazel: Uses both pycodestyle like Python, and also [buildifier](/bazel.html#updating-build-files).


# Manual style fixups

## C/C++: Clang-Format

The [Mandatory platform specific instructions](/from_source.html#mandatory-platform-specific-instructions)
install Drake's required version of ``clang-format``, depending on the platform
(macOS or Ubuntu).

To run ``clang-format``:

```
# For development on macOS:
/usr/local/opt/llvm@9/bin/clang-format -i -style=file [file name]

# For development on Ubuntu:
clang-format-9 -i -style=file [file name]
```

Using ``clang-format`` will modify the entire file that is specified. As an
alternative, you can use ``git clang-format`` on Ubuntu to change only the
portions of a file that you have modified. To run ``git clang-format``:

```
# For development on Ubuntu: format a file that has been staged in git
git clang-format-9 --binary=/usr/bin/clang-format-9 -- [file name]

# For development on Ubuntu: format a file that has been modified but not staged
git clang-format-9 --binary=/usr/bin/clang-format-9 -f -- [file name]
```
