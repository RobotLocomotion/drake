---
title: Documentation Generation Instructions
---

This page contains instructions on how to locally regenerate Drake's website.

The website infrastructure uses a combination of
[Jekyll](https://jekyllrb.com/) for the
[main site](https://drake.mit.edu/) and
[Style Guide](https://drake.mit.edu/styleguide/cppguide.html),
[Doxygen](https://www.doxygen.nl/index.html) for the
[C++ API reference](https://drake.mit.edu/doxygen_cxx/), and
[Sphinx](http://www.sphinx-doc.org/en/stable/) for the
[Python API reference](https://drake.mit.edu/pydrake/).

# Prerequisites

Documentation generation and preview as described in this document are
supported on **Ubuntu only**.

Before getting started, install Drake's prerequisites with the additional
``--with-doc-only`` command line option, i.e.:

```sh
$ sudo setup/ubuntu/install_prereqs.sh --with-doc-only
```

# Previewing changes

To easily preview your changes, run the ``//doc:build --serve`` command.

If you only need to preview a subset of the website, you can speed up the
rebuild by listing a subset of module names on the command line.

When generating the C++ API reference, there is also a ``--quick`` option to
omit the slower steps (e.g., generating dot figures).

```sh
# Read the usage message.
$ bazel run //doc:build -- --help

# Preview the entire site, exactly how it will appear online.
$ bazel run //doc:build -- --serve

# Limiting preview to one major section:
$ bazel run //doc:build -- --serve pages       # Only the main site.
$ bazel run //doc:build -- --serve doxygen     # Only the C++ API reference.
$ bazel run //doc:build -- --serve pydrake     # Only the Python API reference.
$ bazel run //doc:build -- --serve styleguide  # Only the Style Guide.

# Further limiting preview within API references:
$ bazel run //doc:build -- --serve drake.systems    # Only drake/systems/... C++ API.
$ bazel run //doc:build -- --serve pydrake.systems  # Only drake/systems/... Python API.
$ bazel run //doc:build -- --serve {drake,pydrake}.systems  # Both at once.

# Preview a subset of the C++ API reference, omitting expensive images, etc.
$ bazel run //doc:build -- --serve --quick drake.systems.framework
````

To preview using a local branch of the styleguide, set the
[local_repository_override](https://github.com/RobotLocomotion/drake/blob/master/tools/workspace/README.md#exploring-github_archive-changes-from-a-local-clone)
option in ``drake/tools/workspace/styleguide/`` before running the preview.

# Advanced Building

This section containes details aimed at documentation infrastructure
maintainers.

There are in fact five available commands:

```sh
$ bazel run //doc:build               # Entire website (i.e., all of the below).
$ bazel run //doc:pages_build         # Main site only.
$ bazel run //doc/doxygen_cxx:build   # C++ API reference subdir only.
$ bazel run //doc/pydrake:build       # Python API reference subdir only.
$ bazel run //doc/styleguide:build    # Style Guide subdir only.
```

The first command provides options to rebuild subsets of the website, and so
offers one-stop shopping for developers, as explained in the prior section.
The latter commands are used for focused regression testing, and might be
more convenient while modifying the documentation tooling.

Each build command can be run in either "generate" mode (writing the files into
a scratch folder) or "serve" mode (http serving for web browser preview).  The
automated website deployment pipeline uses the former; most developers will use
the latter.

# Testing locally

The website is not part of Drake's default local build nor tests, because it
requires heavy prerequisites to be installed (see ``--with-doc-only`` above).
Therefore, a simple ``bazel test //...`` will not provide any feedback about
local documentation edits.

To check locally that documentation changes pass all build and test rules, run:

```
$ bazel test //doc:manual_tests //doc:manual_binaries //doc/...
```

# Testing in CI

Only the Jenkins builds whose name ends with ``-documentation`` will run the
documentation build steps and related tests.  By default, those builds run on
the Continuous (i.e., post-merge) and Nightly schedules, not on pull requests.
If you would like to check Jenkins results on a pull request, you need to
[schedule an on-demand build](/jenkins.html#scheduling-an-on-demand-build)
by posing a comment

```
@drake-jenkins-bot linux-bionic-unprovisioned-gcc-bazel-experimental-documentation please
```
