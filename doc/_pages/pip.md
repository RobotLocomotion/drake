---
title: Installation via Pip
---

# Pip Packages

Drake publishes pre-compiled binaries as
[PyPI wheel packages](https://pypi.org/project/drake/).  Refer to
[Supported Configurations](/installation.html#supported-configurations)
for additional compatibility details.  Drake wheels require a `pip`
version of `pip >= 20.3`.

To learn about other installation methods, refer to
[Installation and Quickstart](/installation.html).

If you experience any problems or questions with Drake, please
[ask for help on Stack Overflow](/getting_help.html).

Drake binary releases incorporate a pre-compiled version of
[SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) as part of the
[Mathematical Program toolbox](https://drake.mit.edu/doxygen_cxx/group__solvers.html).
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.

## Stable Releases

<div class="warning" markdown="1">
Drake's support for pip has only just recently been introduced (as of October
2021), so still has some known issues (see
[issue #15954](https://github.com/RobotLocomotion/drake/issues/15954)).
Please [let us know](/getting_help.html) if you
experience any additional problems. To fall back to other installation methods
in the meantime, refer to [Installation and Quickstart](/installation.html)
for more choices.
</div>

<div class="warning" markdown="1">
Drake's pip wheels are only published for CPython 3.6 and CPython 3.7 running
on Linux.  In the future, we intend to publish additional builds.
</div>

We recommend installing drake into a
[virtual environment](https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#creating-a-virtual-environment)
directory.  In the example below, we will name that directory ``env``, but you
can choose any name.

Create a virtual environment and install Drake:

```bash
python3 -m venv env
env/bin/pip install --upgrade pip
env/bin/pip install drake
```

Drake requires certain basic runtime libraries from the host linux distribution.

For Ubuntu 18.04, install these additional libraries:

```bash
sudo apt-get install --no-install-recommends \
  libpython3.6 python3-tk libx11-6 libsm6 libxt6 libglib2.0-0
```

Activate the virtual environment:

```bash
source env/bin/activate
````

Refer to [Quickstart](/installation.html#quickstart) for next steps.
