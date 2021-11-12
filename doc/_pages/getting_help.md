---
title: Getting Help
---

# Searching For Your Question

If you need help with Drake, please first review the documentation on this
website for things such as [installation](/installation.html),
the [C++ API](https://drake.mit.edu/doxygen_cxx/index.html), or
[Python bindings](/python_bindings.html).

Please also briefly review
[Drake's open and closed GitHub issues](https://github.com/RobotLocomotion/drake/issues?q=is%3Aissue)
and [StackOverflow posts tagged for Drake](https://stackoverflow.com/questions/tagged/drake)
to see if your issue has been encountered by someone else before.

# Asking Your Question

If you know your question is a bug or feature request, please
[post a GitHub issue](https://github.com/RobotLocomotion/drake/issues/new).

Otherwise, if you are seeking troubleshooting advice or other assistance with
how to apply Drake to specific purposes (e.g., tutorials or a brief example),
then please
[post a question on StackOverflow](https://stackoverflow.com/questions/ask?tags=drake)
with the ``drake`` tag.

If you are actively developing with Drake and may need more active discussions
than what StackOverflow and GitHub may offer, consider asking for access to the
Drake Developers Slack Channel. To do so, please email Russ Tedrake for access.
Please note that this access may not always be readily granted. (Note: If you
are a Drake developer wanting to invite someone, it will be faster to grant
them access if you invite them directly via Slack.)

If you wish to contribute a patch, please see how to [submit a pull request](/developers.html).

# Helpful Information

When reporting an issue, please consider providing the following information
(``helper command in monospace``):

* Operating system (e.g., Ubuntu 18.04 or macOS Monterey)
* Installation method (e.g., pip, apt, binary tar.gz, docker image, or
  rebuilding from source)
* Language you are using (C++ or [Python](/python_bindings.html))
    * If using C++:
      * C++ compiler (e.g., GCC 7.5.0, GCC 9.3.0, Clang 6.0.0)
    * If using Python:
      * Python version (e.g., Python 3.6.7)
      * Python distribution (e.g., Ubuntu apt, macOS homebrew)
* If building from source:
    * Build system (Bazel or CMake)
        * Bazel version (``which bazel; bazel version``)
        * Bazel C++ compiler (``bazel run @drake//common:print_host_settings``)
        * CMake version (``which cmake; cmake --version``)
        * CMake C++ compiler (``cmake -LA <path_to_source_dir> | grep 'CMAKE_.*_COMPILER'``)
    * Git revision (``git rev-parse --short HEAD``)
    * [Building Drake](/from_source.html) vs. downstream project (like [drake_bazel_external](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_bazel_external), [drake_cmake_external](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_external))
* If using binary release:
    * Download URL
    * Contents of ``drake/share/doc/drake/VERSION.TXT``
    * Building downstream project ([drake_cmake_installed](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed))
* If using Binder or Google Colaboratory:
    * A version of your notebook that we can access. Please check this by
      re-visiting your URL in a private browsing window. (You can add
      notebooks to public GitHub Gists.)
    * The version of ``pydrake`` you are using. To obtain the version
      information it, please add the following code to a cell and execute it:

          !cat /opt/drake/share/doc/drake/VERSION.TXT

      The output should look something like ``YYYYMMDDHHMMSS <git-sha>``.
