> Please fill out all quoted sections (`> `) that need a response; feel free
to remove the quoted sections that do not need a response.

> Before you post:

> * Have you reviewed the documentation on
[Getting Help](http://drake.mit.edu/getting_help.html)?
> * Have you reviewed existing GitHub issues and StackOverflow posts?
> * Have you rerun `install_prereqs.sh` on your current commit of `drake`?

## Description

> Please provide a short description: expected behavior, and what actually
happens. If this is an enhancement request, please provide a concrete use case.

## Reproduction

> Please provide a minimal snippet of code or shell commands (<10 lines)
that reproduces the issue. If it is self-contained, but 10-50 lines,
please link to a Gist.

> If your changes to `drake` are more comprehensive, please try to isolate your
problem to the above amount, ensuring that you are using a branch as close to
`master` as possible.

## Debug Information

> If this is an enhancement / feature request that is not platform-specific,
you do not need to include this section. However, please ensure that the
current version of `master` does not already provide what you want.

* What operating system are you running Drake on?
(e.g. Ubuntu 16.04, macOS 10.13)

    > Replace this with your answer.

* Is this an issue with (a) C++, (b) Python, or (c) the distributed tools
(e.g. `drake-visualizer`)?

    > Replace this with your answer.

* Are you (a) building `drake` from source, or (b) from a binary release?

    > Replace this with your answer.

* What version of `python` are you using?

    > Replace this with the output of
    `bash -x -c 'which python2; python2 --version'`

* If you are building from source or using the `drake` C++ library as an
external:

    * What version of Bazel and CMake are you using?

        > Replace this with the output of
        `bash -x -c 'which bazel; bazel version; which cmake; cmake --version'`

    * If you are using Bazel, please provide the host settings in your present
    project workspace:

        > In your workspace, please copy the output of:
        `bazel run @drake//common:print_host_settings`

    * If you are using CMake, please provide the compilers being used in your
    project:

        > First, in your build directory, execute
        `cmake -LA <path_to_source_dir> | grep 'CMAKE_.*_COMPILER'`

        > Given these values, execute `<compiler> --version`, and provide its
        output.

* If you are building `drake`
[from source](http://drake.mit.edu/from_source.html):

    * What git commit of `drake` are you using?

        > Replace this with the output of `git rev-parse --short HEAD`.

    * Are you building `drake` with Bazel or CMake?

        > Replace this with your answer.

    * Are you using `drake` directly, or consuming it as an external?

        > Replace this with your answer.

    * If an external, is it most like
    [`drake_bazel_external`](https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_bazel_external) or
    [`drake_cmake_external`](https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_cmake_external)?

        > Replace this with your answer.

* If you are using `drake` [from a binary release](http://drake.mit.edu/from_binary.html):

    * What are the contents of `.../drake/share/doc/drake/VERSION.txt`?

        > Replace this with your answer.

    * What URL did you download `drake` from?

        > Replace this with your answer.

    * Are you writing a CMake project which is using `drake`, like
    [`drake_cmake_installed`](https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_cmake_installed)?

        > Replace this with your answer.
