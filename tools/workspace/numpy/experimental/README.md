# Experimental NumPy

At present, the following issues / pull requests are needed in order to
enable providing user-defined dtypes for `AutoDiffXd`, `Expression`, etc:

*   [Pull numpy/numpy#10898](https://github.com/numpy/numpy/pull/10898)
Required so that a non-trivial copy constructor is used. Prevents aliasing
(which would cause double-free errors) for `AutoDiffXd`.
*   [Issue numpy/numpy#9351](https://github.com/numpy/numpy/issues/9351)
Operations like `np.trace` may get caught in an infinite loop. The PRs that
resolve this were incorporated into `>= v1.13.1`.

## Building

To generate a `*.whl` file in `./build/`, that can then be redistributed
/ used in Bazel.

### Ubuntu

To build an experimental NumPy for Ubuntu 16.04, run:

    ./build_ubuntu.sh

### Mac

Ensure that you have the requirements equivalent to those in `Dockerfile`, then
run:

    ./build_mac.sh
