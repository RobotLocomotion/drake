---
title: Installation via APT (Ubuntu)
---

# APT Packages

Drake publishes pre-compiled binaries as APT packages (``*.deb``) for the
Ubuntu 18.04 (Bionic) and Ubuntu 20.04 (Focal) operating systems. Refer to
[Supported Configurations](/installation.html#supported-configurations)
for additional compatibility details.

To learn about other installation methods, refer to
[Installation and Quickstart](/installation.html).

If you experience any problems or questions with Drake, please
[ask for help on Stack Overflow](/getting_help.html).

Drake binary releases incorporate a pre-compiled version of
[SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) as part of the
[Mathematical Program toolbox](https://drake.mit.edu/doxygen_cxx/group__solvers.html).
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.

## Stable Releases

To add the Drake APT repository to your machine and install the `drake-dev` package,
please do the following in order:

If you are using a [minimal](https://wiki.ubuntu.com/Minimal) cloud or
container image, you may need to install the following packages:

```bash
sudo apt-get update
sudo apt-get install --no-install-recommends \
  ca-certificates gnupg lsb-release wget
```

Download a copy of the Drake GPG signing key and add it to an APT trusted keychain:

```bash
wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
  | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
```

Add the Drake repository to your APT sources list:

```bash
echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null
```

Update your local APT package index and install the `drake-dev` package:

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

Refer to [Quickstart](/installation.html#quickstart) for next steps.
