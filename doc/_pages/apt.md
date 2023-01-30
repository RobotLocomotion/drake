---
title: Installation via APT (Ubuntu)
---

# APT Packages

Drake publishes pre-compiled binaries as APT packages (``*.deb``) for the
Ubuntu 20.04 (Focal) and Ubuntu 22.04 (Jammy) operating systems.
Refer to
[Supported Configurations](/installation.html#supported-configurations)
for additional compatibility details.

To learn about other installation methods, refer to
[Installation and Quickstart](/installation.html).

If you experience any problems with or have questions about Drake, please
[ask for help](/getting_help.html).

Drake binary releases incorporate a pre-compiled version of
[SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) as part of the
[Mathematical Program toolbox](https://drake.mit.edu/doxygen_cxx/group__solvers.html).
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.

Drake's apt packages do not support the Gurobi solver. To use
Gurobi, you will need to build Drake from source following the instructions
in [Source Installation](/from_source.html).

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
  export PATH="/opt/drake/bin${PATH:+:${PATH}}"
  export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
  ```

Refer to [Quickstart](/installation.html#quickstart) for next steps.

## Nightly Releases

Unsigned nightly apt packages of Drake for Ubuntu 20.04 (Focal) and Ubuntu 22.04
(Jammy) are available to download at:

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-dev_latest-1_amd64-focal.deb](https://drake-packages.csail.mit.edu/drake/nightly/drake-dev_latest-1_amd64-focal.deb)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-dev_latest-1_amd64-jammy.deb](https://drake-packages.csail.mit.edu/drake/nightly/drake-dev_latest-1_amd64-jammy.deb)

Older packages for specific dates are available by replacing ``latest`` with an
8-digit date, e.g., ``20220721`` for July 21st, 2022.

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-dev_0.0.20220721-1_amd64-focal.deb](https://drake-packages.csail.mit.edu/drake/nightly/drake-dev_0.0.20220721-1_amd64-focal.deb)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-dev_0.0.20220721-1_amd64-jammy.deb](https://drake-packages.csail.mit.edu/drake/nightly/drake-dev_0.0.20220721-1_amd64-jammy.deb)

Nightly packages are retained for 56 days from their date of creation.

For installing a nightly apt package, download the archive and install it
directly:

  ```bash
  wget https://drake-packages.csail.mit.edu/drake/nightly/drake-dev_latest-1_amd64-focal.deb
  sudo apt-get install --no-install-recommends ./drake-dev_latest-1_amd64-focal.deb
  ```
