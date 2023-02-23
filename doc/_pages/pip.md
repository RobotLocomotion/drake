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

If you experience any problems with or have questions about Drake, please
[ask for help](/getting_help.html).

Drake binary releases incorporate a pre-compiled version of
[SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) as part of the
[Mathematical Program toolbox](https://drake.mit.edu/doxygen_cxx/group__solvers.html).
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.

Drake's pip packages do not support the Gurobi solver. To use
Gurobi, you will need to build Drake from source following the instructions
in [Source Installation](/from_source.html).

## Stable Releases

<div class="warning" markdown="1">
Drake does not support the Python environment supplied by Anaconda. Before
installing or using Drake, please `conda deactivate` (repeatedly, until even
the conda base environment has been deactivated) such that none of the paths
reported `which -a python python3 pip pip3` refer to conda.
Note that Miniconda seems to work fine; it's only Anaconda that has caused
problems for some users.
</div>

<div class="warning" markdown="1">
Drake's support for pip has a few known issues (see
[issue #15954](https://github.com/RobotLocomotion/drake/issues/15954)).
Please [let us know](/getting_help.html) if you
experience any additional problems. To fall back to other installation methods
in the meantime, refer to [Installation and Quickstart](/installation.html)
for more choices.
</div>

We recommend installing drake into a
[virtual environment](https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#creating-a-virtual-environment)
directory.  In the example below, we will name that directory ``env``, but you
can choose any name.

For macOS, ensure that you're using Homebrew Python (not Apple's system Python).

Create a virtual environment, install Drake, and activate the environment:

```bash
python3 -m venv env
env/bin/pip install --upgrade pip
env/bin/pip install drake
source env/bin/activate
````

Refer to [Quickstart](/installation.html#quickstart) for next steps.

## Nightly Releases

Binary packages of Drake for Ubuntu 20.04 (Focal), Ubuntu 22.04 (Jammy), and
macOS are generated nightly and are available to download at:

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp38-cp38-manylinux_2_31_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp38-cp38-manylinux_2_31_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp39-cp39-manylinux_2_31_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp39-cp39-manylinux_2_31_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp310-cp310-manylinux_2_31_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp310-cp310-manylinux_2_31_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp311-cp311-manylinux_2_31_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp311-cp311-manylinux_2_31_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp311-cp311-macosx_12_0_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp311-cp311-macosx_12_0_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp311-cp311-macosx_12_0_arm64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp311-cp311-macosx_12_0_arm64.whl)

Older packages for specific dates are available by replacing ``latest`` with an
8-digit date, e.g., ``20230215`` for February 15th, 2023.  The version number to
replace ``latest`` with follows the pattern ``0.0.YYYYMMDD``.

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp38-cp38-manylinux_2_31_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp38-cp38-manylinux_2_31_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp39-cp39-manylinux_2_31_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp39-cp39-manylinux_2_31_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp310-cp310-manylinux_2_31_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp310-cp310-manylinux_2_31_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp311-cp311-manylinux_2_31_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp311-cp311-manylinux_2_31_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp311-cp311-macosx_12_0_x86_64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp311-cp311-macosx_12_0_x86_64.whl)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp311-cp311-macosx_12_0_arm64.whl](https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20230215-cp311-cp311-macosx_12_0_arm64.whl)

Nightly wheels are retained for 56 days from their date of creation.

To install nightly wheel, install from the URL directly:

```bash
# Example for python 3.8 (cp38-cp38).
python3 -m venv env
env/bin/pip install --upgrade pip
env/bin/pip install https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-cp38-cp38-manylinux_2_31_x86_64.whl
source env/bin/activate
```
