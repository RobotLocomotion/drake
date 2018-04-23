# Packaging NumPy

In order for `pydrake` to support `numpy` arrays of non-doubles in a way
semantically similar to arrays of doubles, we need to use `numpy` upstream
releases of at least version 1.15.0. Because it is not generally available in
binary form, Drake distributes its own binary `*.whl`s as a convenience.

The required issues / pull requests for these features are:

*   [Pull numpy/numpy#10898](https://github.com/numpy/numpy/pull/10898)
Required so that a non-trivial copy constructor is used. Prevents aliasing
(which would cause double-free errors) for `AutoDiffXd`.
*   [Issue numpy/numpy#9351](https://github.com/numpy/numpy/issues/9351)
Operations like `np.trace` may get caught in an infinite loop. The PRs that
resolve this were incorporated into `>= v1.13.1`.
*   [Pull numpy/numpy#11075](https://github.com/numpy/numpy/issues/11075)
Required so that scalar-to-array assignment correctly initializes memory when
casting (e.g. `x = np.zeros(2, dtype=AutoDiffXd); x[:] = 0.`).

## Building

The following sections show platform-specific instructions to generate a `*.whl`
file in `./build/`, that can then be used via Bazel.

### Ubuntu

To build NumPy for Ubuntu 16.04, run:

    ./build_ubuntu.sh

### Mac

Ensure that you have the prerequisites to build NumPy:

    brew bundle --file="./Brewfile"

Then run:

    ./build_mac.sh

## Updating

When updating this build of NumPy, please do the following:

### Developing

1. In `./build_direct.sh`, set `repo` to your fork of NumPy, and `commit` to
whatever commmit you're experimenting with. Then use the above steps to
generate archives.
    * **NOTE**: If you want more rapid prototyping / debugging, you should
    build and install `numpy` to a local directory (e.g.
    `~/.local/numpy_debug/{whatever}`), update your `PYTHONPATH`, and then
    change `numpy_py_repository` in `.../numpy/repository.bzl` to be
    effectively be a no-op so the library is used from your `PYTHONPATH`. In
    this way, you can build with debug versions of CPython + NumPy
    ([example for Ubuntu 16.04](https://gist.github.com/EricCousineau-TRI/ce79d3265bb72934267e24ddc8c623bc#file-cpython_dbg_valgrind-sh)).
1. Test your code locally on your machine, using these build steps to produce an
archive.
1. Upload your *temporary* archives to something like a temporary Git
repository, so you can update `.../numpy/repository.bzl` with the versions you
need. Update your branch to use these archives.

### Submitting

1. Submit your PR to Drake, and ensure your PR passes on the Drake CI for both
Ubuntu *and* Mac.
    * You will need to also update the variable `expected_version` in
    `.../numpy/test/numpy_install_test.py` for the install tests to all pass.
1. Once you are confident that these are the NumPy changes necessary, submit an
upstream PR to [`numpy`](https://github.com/numpy/numpy), and work with the
authors to ensure it is something that can land on `master`.
    * If they suggest any changes or workarounds, ensure that you reflect those
    changes in your Drake PR, and iterate through review.
1. Once you are confident that your upstream PR will land at some point, and
the general feature review is complete:
    * In `./build_direct.sh`, set `repo` back to upstream `numpy`, and set
    `commit` to your PR, e.g. `pull/10898/head`.
    * Add `@jamiesnape` to the review, and ask if he can build and deploy the
    `*.whl` binaries to S3.
1. Use the URLs that Jamie provides in `.../numpy/repository.bzl`, and ensure
you run your PR through CI once more.

[//]: # "TODO(eric.cousineau): See if there is a means to automate uploading"
[//]: # "when Jamie gives the OK."
