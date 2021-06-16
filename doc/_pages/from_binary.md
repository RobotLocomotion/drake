---
title: Binary installation (macOS, Ubuntu)
---

# Nightly Releases

Binary packages of Drake for Ubuntu 18.04 (Bionic), Ubuntu 20.04 (Focal) and
Mac are generated nightly and are tagged ``latest`` or by ``yyyymmdd``. Latest
releases are available to download at:

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-focal.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-focal.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac.tar.gz)

Releases for specific days are available as follows:
`https://drake-packages.csail.mit.edu/drake/nightly/drake-yyyymmdd-bionic|focal|mac.tar.gz`

For example: [https://drake-packages.csail.mit.edu/drake/nightly/drake-20191026-bionic.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-20191026-bionic.tar.gz)


Note that Drake no longer supports Ubuntu 16.04 (Xenial), but older packages are available here:

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-20191026-xenial.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-20191026-xenial.tar.gz)

Individual packages are archived two years from their date of creation.

Note that for release v0.30.0 and later, Ubuntu binaries require support for
Intel's AVX2 and FMA instruction sets which were introduced with the Haswell
architecture in 2013 with substantial performance improvements in the Broadwell
architecture in 2014. Drake is compiled with `-march=broadwell` to exploit these
instructions (that also works for Haswell machines). Drake can be used on older
machines if necessary by building from source with that flag removed.

For the compilers used to produce these releases, see
[Binary Packages](/developers.html#binary-packages).

Drake binary releases incorporate a pre-compiled version of
[SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) as part of the
[Mathematical Program toolbox](https://drake.mit.edu/doxygen_cxx/group__solvers.html).
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.


## Example Usage

An example of how to use the nightly binaries is shown in this
[example CMake project](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed).

## Binary Installation for Python

To use Python bindings, see [Binary Installation for Python](/python_bindings.html#installation).

## Docker Images

You may also use binary releases in Docker images. See
[Using the Drake Docker Images From Docker Hub](/docker.html)
for more information.

## Experimental Packages

Drake maintainers may build "experimental" packages on demand using Jenkins by
following [these instructions](/jenkins.html#building-binary-packages-on-demand).

## Historical Note

Older releases were built around substantial MATLAB support, and are
described on [release notes page](/release_notes/older_releases.html).

# APT Packages for Monthly Tagged Releases

APT packages are currently available for the Ubuntu 18.04 (Bionic) and
Ubuntu 20.04 (Focal) operating systems on x86 64-bit architectures.

## To Install the Packages

To add the Drake APT repository to your machine and install the `drake-dev` package,
please do the following in order:

1. If you are using a [minimal](https://wiki.ubuntu.com/Minimal) cloud or
   container image, you may need to install the following packages:
   ```bash
   sudo apt-get update
   sudo apt-get install --no-install-recommends \
     ca-certificates gnupg lsb-release wget
   ```

2. Download a copy of the Drake GPG signing key and add it to an APT trusted keychain:
   ```bash
   wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
     | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
   ```

3. Add the Drake repository to your APT sources list:
   ```bash
   echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
     | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null
   ```

4. Update your local APT package index and install the `drake-dev` package:
   ```bash
   sudo apt-get update
   sudo apt-get install --no-install-recommends drake-dev
   ```

Most content installs to `/opt/drake`, so setting the following environment
variables may be useful:
  ```bash
  export LD_LIBRARY_PATH="/opt/drake/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PATH="/opt/drake/bin${PATH:+:${PATH}}"
  export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
  ```
