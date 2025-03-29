---
title: Installation via Pip (Python)
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

<div class="note" markdown="1">
Please note the following compatibility requirements for using Drake via Python:
* Drake's pip packages do not support the Gurobi solver. To use
Gurobi, you will need to build Drake from source following the instructions
in [Source Installation](/from_source.html).
* Drake is not tested regularly with Anaconda, so if you are using Anaconda you
may experience compatibility hiccups; when asking for help, be sure to mention
that Conda is involved.
* For macOS, ensure that you're using Homebrew Python (not Apple's system Python).
</div>

We recommend installing drake into a
[virtual environment](https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#creating-a-virtual-environment)
directory.  In the examples below, we will name that directory ``env``, but you
can choose any name.

## Stable Releases

Create a virtual environment, install Drake, and activate the environment:

```bash
python3 -m venv env
env/bin/pip install drake
source env/bin/activate
```

Refer to [Quickstart](/installation.html#quickstart) for next steps.

## Nightly Releases

Wheel packages for Ubuntu 22.04 (Jammy), Ubuntu 24.04 (Noble), and macOS are
published nightly at a [PEP 503](https://peps.python.org/pep-0503/) index url
[https://drake-packages.csail.mit.edu/whl/nightly/](https://drake-packages.csail.mit.edu/whl/nightly/).

- Nightly wheel version numbers are created as `0.0.YYYYMMDD`, e.g.,
  `0.0.20240221` for February 21st, 2024.
- Nightly wheel packages are retained for 56 days from their date of creation,
  and drop out of the index after 48 days.

To install a specific nightly wheel using `pip`, replace `YYYYMMDD` with the
desired date:

```bash
python3 -m venv env
env/bin/pip install \
    --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/ \
    'drake==0.0.YYYYMMDD'
source env/bin/activate
```

Or, to install today's most recent nightly wheel using `pip`, specify `<0.1`
instead of `==0.0.YYYYMMDD` as shown below:

```bash
python3 -m venv env
env/bin/pip install \
    --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/ \
    'drake<0.1'
source env/bin/activate
```
