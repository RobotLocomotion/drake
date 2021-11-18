---
title: Source Installation
---

# Supported Configurations

The following table shows the configurations and platforms that Drake
officially supports. Supported configurations are tested in continuous
integration. Any other configurations are provided on a best-effort basis.

| Operating System ⁽⁴⁾             | Architecture | Python  | Bazel | CMake | C/C++ Compiler ⁽⁵⁾                 | Java                          |
|----------------------------------|--------------|---------|-------|-------|------------------------------------|-------------------------------|
| Ubuntu 18.04 LTS (Bionic Beaver) | x86_64 ⁽¹⁾   | 3.6 ⁽³⁾ | 4.2   | 3.10  | GCC 7.5 (default) or Clang 9   | OpenJDK 11                    |
| Ubuntu 20.04 LTS (Focal Fossa)   | x86_64 ⁽¹⁾   | 3.8 ⁽³⁾ | 4.2   | 3.16  | GCC 9.3 (default) or Clang 9   | OpenJDK 11                    |
| macOS Big Sur (11)               | x86_64 ⁽²⁾   | 3.9 ⁽³⁾ | 4.2   | 3.19  | Apple LLVM 12.0.0 (Xcode 12.4) | AdoptOpenJDK 15 (HotSpot JVM) |
| macOS Monterey (12)              | x86_64 ⁽²⁾   | 3.9 ⁽³⁾ | 4.2   | 3.19  | Apple LLVM 12.0.0 (Xcode 12.4) | AdoptOpenJDK 15 (HotSpot JVM) |

⁽¹⁾ Drake Ubuntu builds assume support for Intel's AVX2 and FMA instructions,
introduced with the Haswell architecture in 2013 with substantial performance
improvements in the Broadwell architecture in 2014. Drake is compiled with
`-march=broadwell` to exploit these instructions (that also works for Haswell
machines). Drake can be used on older machines if necessary by building from
source with that flag removed.

⁽²⁾ Running Drake under Rosetta 2 emulation on arm64 is not supported. Plans
for any future arm64 support on macOS and/or Ubuntu are discussed in
[issue #13514](https://github.com/RobotLocomotion/drake/issues/13514).

⁽³⁾ CPython is the only Python implementation supported.

⁽⁴⁾ Drake features that perform image rendering (e.g., camera simulation)
require a working display server.  Most personal computers will have this
already built in, but some cloud or docker environments may not.

⁽⁵⁾ Drake requires a compiler running in C++17 or C++20 mode.

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

# Mandatory platform specific instructions

Before running the build, you must follow some one-time platform-specific
setup steps:

* [macOS](/mac.html)
* [Ubuntu](/ubuntu.html)

See [above](#supported-configurations)
for the configurations and platforms that Drake officially supports.
All else being equal, we would recommend developers use Ubuntu Bionic.

# Build with Bazel

For instructions, jump to [Using Bazel](/bazel.html#developing-drake-using-bazel), or check out the
full details at:

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

Please note the additional CMake options which affect the Python bindings:

* ``-DWITH_GUROBI={ON, [OFF]}`` - Build with Gurobi enabled.
* ``-DWITH_MOSEK={ON, [OFF]}`` - Build with MOSEK enabled.
* ``-DWITH_SNOPT={ON, [OFF]}`` - Build with SNOPT enabled.

``{...}`` means a list of options, and the option surrounded by ``[...]`` is
the default option. An example of building ``pydrake`` with both Gurobi and
MOSEK, without building tests:

```bash
cmake -DWITH_GUROBI=ON -DWITH_MOSEK=ON ../drake
```

You will also need to have your ``PYTHONPATH`` configured correctly.

*Ubuntu 18.04 (Bionic):*

```bash
cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.6/site-packages:${PYTHONPATH}
```

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
