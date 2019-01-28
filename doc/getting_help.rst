.. _getting_help:

************
Getting Help
************

Searching For Your Question
===========================

If you need help with Drake, please first review the documentation on this
website for things such as :ref:`installation <installation_and_quick_start>`,
`the C++ API <doxygen_cxx/index.html#://>`_, or
:ref:`Python bindings <python-bindings>`.

Please also briefly review
`Drake's open and closed GitHub issues <https://github.com/RobotLocomotion/drake/issues?q=is%3Aissue>`_
and `StackOverflow posts tagged for Drake <https://stackoverflow.com/questions/tagged/drake>`_
to see if your issue has been encountered by someone else before.

Asking Your Question
====================

If you know your question is a bug or feature request, please
`post a GitHub issue <https://github.com/RobotLocomotion/drake/issues/new>`_.

Otherwise, if you are seeking assistance (e.g. tutorials or a brief example),
please `post a question on StackOverflow
<https://stackoverflow.com/questions/ask?tags=drake>`_ with the ``drake`` tag.

If you are actively developing with Drake and may need more active discussions
than what StackOverflow and GitHub may offer, consider asking for access to the
Drake Developers Slack Channel. To do so, please email Russ Tedrake for access.
Please note that this access may not always be readily granted.

If you wish to contribute a patch, please see how to :ref:`submit a pull request
<pull_request>`.

Helpful Information
-------------------

When reporting an issue, please consider providing the following information
(*examples in italics*):

*   Operating system (*Ubuntu 16.04 / 18.04, macOS 10.14*)
*   Language (C++, :ref:`Python <python-bindings>`)

    -   C++ compiler (*GCC 7.3.0, Clang 6.0.0*)
    -   Python version (*Python 2.7.15, Python 3.6.7*)
    -   Python distribution (*apt, homebrew*)

*   If building from source:

    -   Build system (Bazel, CMake*)

        +  Bazel version (``which bazel; bazel version``)
        +  Bazel C++ compiler (``bazel run @drake//common:print_host_settings``)
        +  CMake version (``which cmake; cmake --version``)
        +  CMake C++ compiler (``cmake -LA <path_to_source_dir> | grep 'CMAKE_.*_COMPILER'``)

    -  Git revision (``git rev-parse --short HEAD``)
    -  :ref:`Building Drake <build_from_source>` vs. downstream project (like `drake_bazel_external <https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_bazel_external>`_, `drake_cmake_external <https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_cmake_external>`_)

*   If using binary release:

    -   Download URL
    -   Contents of ``drake/share/doc/drake/VERSION.txt``
    -   Building downstream project (`drake_cmake_installed <https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_cmake_installed>`_)

Older Sources
=============

Some information was previously on a
`mailing list <http://mailman.mit.edu/mailman/listinfo/drake-users>`_.
Please do not use these resources, as they are now inactive.
