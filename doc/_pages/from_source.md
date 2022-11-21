---
title: Source Installation
---

# Supported Configurations

The following table shows the configurations and platforms that Drake
officially supports. Supported configurations are tested in continuous
integration. Any other configurations are provided on a best-effort basis.

<!-- The operating system requirements should match those listed in both the
     root CMakeLists.txt and tools/workspace/os.bzl. -->
<!-- The minimum compiler versions should match those listed in both the root
     CMakeLists.txt and tools/workspace/cc/repository.bzl. -->

| Operating System ⁽²⁾               | Architecture | Python ⁽¹⁾ | Bazel | CMake | C/C++ Compiler ⁽³⁾           | Java                          |
|------------------------------------|--------------|------------|-------|-------|------------------------------|-------------------------------|
| Ubuntu 20.04 LTS (Focal Fossa)     | x86_64       | 3.8        | 5.3   | 3.16  | GCC 9 (default) or Clang 12  | OpenJDK 11                    |
| Ubuntu 22.04 LTS (Jammy Jellyfish) | x86_64       | 3.10       | 5.3   | 3.22  | GCC 11 (default) or Clang 12 | OpenJDK 11                    |
| macOS Monterey (12)                | x86_64       | 3.10       | 5.3   | 3.24  | Apple LLVM 14 (Xcode 14)     | AdoptOpenJDK 16 (HotSpot JVM) |
| macOS Monterey (12)                | arm64        | 3.10       | 5.3   | 3.24  | Apple LLVM 14 (Xcode 14)     | AdoptOpenJDK 16 (HotSpot JVM) |

⁽¹⁾ CPython is the only Python implementation supported.

⁽²⁾ Drake features that perform image rendering (e.g., camera simulation)
require a working display server.  Most personal computers will have this
already built in, but some cloud or docker environments may require extra
setup steps.

⁽³⁾ Drake requires a compiler running in C++17 or C++20 mode.

# Getting Drake

We recommend that you [setup SSH access to GitHub.com](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/)
to avoid needing to type your password each time you access it. The following
instructions assume you have uploaded your public SSH key to your GitHub
account.

Now run:

```
git clone git@github.com:RobotLocomotion/drake.git
```

Note: we suggest you keep the default clone directory name (``drake``) and not
rename it (such as ``drake2``).  The CLion integration will suffer if the
checkout directory is not named ``drake``.  (See [CLion IDE setup](clion.html) for details.)

Note: the build process may encounter problems if you have unusual characters
like parentheses in the absolute path to the drake directory
(see [#394](https://github.com/RobotLocomotion/drake/issues/394)).

The above ``git clone`` command will configure Drake's primary repository as a
remote called ``origin``. We recommend that you configure your fork of Drake's
primary repository as the ``origin`` remote and Drake's primary repository as
the ``upstream`` remote. This can be done by executing the following commands:

```
cd drake
git remote set-url origin git@github.com:[your github user name]/drake.git
git remote add upstream git@github.com:RobotLocomotion/drake.git
git remote set-url --push upstream no_push
```

# Mandatory platform-specific instructions

Before running the build, you must follow some one-time platform-specific
setup steps:

* [macOS](/mac.html)
* [Ubuntu](/ubuntu.html)

See [above](#supported-configurations)
for the configurations and platforms that Drake officially supports.
All else being equal, we would recommend developers use Ubuntu Focal.

# Build with Bazel

For instructions, jump to
[Developing Drake using Bazel](/bazel.html#developing-drake-using-bazel),
or check out the full details at:

* [Bazel build system](/bazel.html)

## Building the Python Bindings

To use the Python bindings from Drake externally, we recommend using CMake.
As an example:

```bash
git clone https://github.com/RobotLocomotion/drake.git
mkdir drake-build
cd drake-build
cmake ../drake
make -j
```

Note that a concurrency limit passed to `make` (e.g., `make -j 2`) has almost no
effect on the Drake build. You might need to add a bazel configuration dotfile
to your home directory if your build is running out of memory. See the
[troubleshooting](/troubleshooting.html#build-oom) page for details.

Please note the additional CMake options which affect the Python bindings:

* ``-DWITH_GUROBI={ON, [OFF]}`` - Build with Gurobi enabled.
* ``-DWITH_MOSEK={ON, [OFF]}`` - Build with MOSEK™ enabled.
* ``-DWITH_SNOPT={ON, [OFF]}`` - Build with SNOPT enabled.

``{...}`` means a list of options, and the option surrounded by ``[...]`` is
the default option. An example of building ``pydrake`` with both Gurobi and
MOSEK™, without building tests:

```bash
cmake -DWITH_GUROBI=ON -DWITH_MOSEK=ON ../drake
```

You will also need to have your ``PYTHONPATH`` configured correctly.

*Ubuntu 20.04 (Focal):*

```bash
cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.8/site-packages:${PYTHONPATH}
```
*macOS:*

```bash
cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.9/site-packages:${PYTHONPATH}
```
