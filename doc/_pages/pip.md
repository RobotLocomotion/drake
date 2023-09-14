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

<div class="note" markdown="1">
Drake is not tested regularly with Anaconda, so if you are using Anaconda you
may experience compatibility hiccups; when asking for help, be sure to mention
that Conda is involved.
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
macOS are generated nightly.  A [PEP 503](https://peps.python.org/pep-0503/)
compliant extra index url is uploaded to the drake-packages s3 url
[https://drake-packages.csail.mit.edu/whl/nightly/](https://drake-packages.csail.mit.edu/whl/nightly/).

- Nightly wheel version numbers are created as `0.0.YYYYMMDD`, e.g.,
  `0.0.20230914` for Septemper 14th, 2023.
- The index is updated each morning as the final step of every successful wheel
  build, and advertises the listing of wheels available for the last 48 days.
  Once a nightly wheel is older than 48 days, it will not be available for
  download in the future.  Use a stable release tag if you need something to
  exist longer, or mirror the wheel yourself.

To install a nightly wheel using `pip`, replace `YYYYMMDD` with the desired
date:

```bash
python3 -m venv env
env/bin/pip install --upgrade pip
env/bin/pip install \
    --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/ \
    'drake==0.0.YYYYMMDD'
```

<div class="note" markdown="1">
Since we only host an index for `drake` artifacts, we must use
`--extra-index-url` to enable
`https://drake-packages.csail.mit.edu/whl/nightly/` to be considered _in
addition to [PyPI](https://pypi.org/)_.  The version numbering chosen for
nightly wheels accounts for this, if you want to install the latest available
nightly wheel for your platform:

```bash
env/bin/pip install \
    --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/ \
    'drake<0.1'
```

If you just install `drake`, the version numbers for the stable releases
for the [drake PyPI project](https://pypi.org/project/drake/) will be larger
than any `0.0.YYYYMMDD` version in the extra index, and you will **not** be
installing a nightly wheel artifact!
</div>
