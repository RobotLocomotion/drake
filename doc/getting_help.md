---
title: Getting Help
---

#### Searching For Your Question

If you need help with Drake, please first review the documentation on this
website for things such as [installation](/installation.html),
the [C++ API](https://drake.mit.edu/doxygen_cxx/index.html#://), or
[Python bindings](/python_bindings.html).

Please also briefly review
[Drake's open and closed GitHub issues](https://github.com/RobotLocomotion/drake/issues?q=is%3Aissue)
and [StackOverflow posts tagged for Drake](https://stackoverflow.com/questions/tagged/drake)
to see if your issue has been encountered by someone else before.

#### Asking Your Question

If you know your question is a bug or feature request, please
[post a GitHub issue](https://github.com/RobotLocomotion/drake/issues/new).

Otherwise, if you are seeking assistance (e.g. tutorials or a brief example),
please [post a question on StackOverflow](https://stackoverflow.com/questions/ask?tags=drake)
with the ``drake`` tag.

If you are actively developing with Drake and may need more active discussions
than what StackOverflow and GitHub may offer, consider asking for access to the
Drake Developers Slack Channel. To do so, please email Russ Tedrake for access.
Please note that this access may not always be readily granted. (Note: If you
are a Drake developer wanting to invite someone, it will be faster to grant
them access if you invite them directly via Slack.)

If you wish to contribute a patch, please see how to [submit a pull request](/developers.html#pull-request).

#### Helpful Information

When reporting an issue, please consider providing the following information
(*examples in italics*, ``helper command in monospace``):

* Operating system (*Ubuntu 18.04, macOS Catalina*)
* Language (C++, [Python](/python-bindings.html))
    * C++ compiler (*GCC 7.5.0, GCC 9.3.0, Clang 6.0.0*)
    * Python version (*Python 3.6.7*)
    * Python distribution (*apt, homebrew*)
* If building from source:
    * Build system (Bazel, CMake)
        * Bazel version (``which bazel; bazel version``)
        * Bazel C++ compiler (``bazel run @drake//common:print_host_settings``)
        * CMake version (``which cmake; cmake --version``)
        * CMake C++ compiler (``cmake -LA <path_to_source_dir> | grep 'CMAKE_.*_COMPILER'``)
    * Git revision (``git rev-parse --short HEAD``)
    * [Building Drake](/from_source.html) vs. downstream project (like [drake_bazel_external](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_bazel_external), [drake_cmake_external](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_external))
* If using binary release:
    * Download URL
    * Contents of ``drake/share/doc/drake/VERSION.txt``
    * Building downstream project ([drake_cmake_installed](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed))

#### Older Sources

Some information was previously on a
[mailing list](http://mailman.mit.edu/mailman/listinfo/drake-users).
Please do not use these resources, as they are now inactive.
