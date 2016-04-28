**************
For Developers
**************

If you have improvements to Drake, send us your pull requests!

The standard github workflow is to fork the drake repository into your
own github account, push your changes into a branch on that account,
then (when your code is definitely ready) open a `pull request
<https://help.github.com/articles/using-pull-requests/>`_ via the
github website.  Please read on for information on testing, code
review, and code style.

We would like to hear about your success stories if you've used
Drake in your own projects.  Please consider contributing to our :doc:`gallery`
by editing ``gallery.rst`` in the ``drake/doc/`` directory and submitting a pull
request.

Licensing
=========

**Important note:** Drake is an open source project licensed under
extremely flexible terms intended to encourage use by anyone, for any
purpose. When you make a contribution to the Drake project, you are
agreeing to do so under those same terms.

Testing
=======

When you create a pull request, unit and regression tests will
automatically run on a number of build servers.  You can run those
tests locally by running ``make test`` from the command line. Note
that there are a lot of computationally demanding tests and this could
run for a few hours depending on your machine.

Your change must include unit tests that protect it against regressions,
and those tests must pass on all platforms supported by Drake.

**For C++ changes,** please use the googletest framework. It is already
available in the superbuild.

**For MATLAB changes,** please write an example and add it as a test in
directory CMakeLists.txt using add_matlab_test().  ctest will consider
the test failed if it times out or calls ``error``.

.. _supported-configurations:

Supported Configurations
========================

The following table shows the configurations and platforms that Drake
officially supports. Supported configurations are tested in continuous
integration. All other configurations are provided on a best-effort basis.

On Ubuntu and OS X, only the "Unix Makefiles" CMake generator is supported.
On Windows, only the "Visual Studio 14 2015" and "Visual Studio 14 2015 Win64"
CMake generators are supported.

+-----------------------------------------+--------------------+-------------------+---------+
| Operating System                        | Compiler           | Superbuild Deps   | Build   |
+=========================================+====================+===================+=========+
| Ubuntu 14.04 LTS                        | GCC 4.9            | Default           | Debug   |
|                                         |                    |                   +---------+
|                                         |                    |                   | Release |
|                                         |                    +-------------------+---------+
|                                         |                    | Default + MATLAB  | Release |
|                                         +--------------------+-------------------+---------+
|                                         | Clang 3.7          | Default           | Debug   |
|                                         |                    |                   +---------+
|                                         |                    |                   | Release |
+-----------------------------------------+--------------------+-------------------+---------+
| | Windows Server 2012 R2 or Windows 8.1 | MSVC 14 32-bit     | Default           | Debug   |
| | Visual Studio 2015 (any edition)      |                    |                   +---------+
|                                         |                    |                   | Release |
|                                         +--------------------+-------------------+---------+
|                                         | MSVC 14 64-bit     | Default           | Debug   |
|                                         |                    |                   +---------+
|                                         |                    |                   | Release |
+-----------------------------------------+--------------------+-------------------+---------+
| OS X 10.10                              | Apple Clang 7.0    | Default           | Debug   |
|                                         |                    |                   +---------+
|                                         |                    |                   | Release |
+-----------------------------------------+--------------------+-------------------+---------+

Official support for MATLAB on Windows and OS X is planned for 2016 Q2.

Code Review
===========

For complex changes, especially those that will span multiple PRs, please
open a GitHub issue and solicit design feedback before you invest a lot of
time in code.

Be prepared to engage in active code review on your pull requests.  The Drake
code review process has two phases: feature review and platform review. You
are responsible for finding reviewers, and for providing them the information
they need to review your change effectively. If a reviewer asks you for more
information, that is a sign you should add more documentation to your PR.

We use https://reviewable.io for code reviews. You can sign in for free with
your GitHub identity. Before your first code review, please take a look at
:doc:`reviewable`.

**Feature Review.** After creating your pull request, assign it to someone
else on your team for feature review. Choose the person most familiar
with the context of your pull request. This reviewer is responsible for
protecting your team by inspecting for bugs, for test coverage, and for
alignment with the team's goals. During this review, you and your reviewer
should also strive to minimize the number of changes that will be necessary
in platform review.

**Platform Review.** After your feature reviewer has signed off on your change,
reassign it to a Drake owner for platform review. The owner will inspect for
architectural compatibility, stability, performance, test coverage, and style.

The following GitHub users are Drake owners. If possible, seek platform review
from an owner who has previously reviewed related changes. Shared context will
make the review faster.

- @david-german-tri (Toyota Research Institute)
- @ggould-tri (Toyota Research Institute)
- @jwnimmer-tri (Toyota Research Institute)
- @psiorx (MIT)
- @sherm1 (Toyota Research Institute)
- @RussTedrake (MIT / Toyota Research Institute)

**Merge.** If you have write access to RobotLocomotion/drake, a green
"Merge Pull Request" button will appear when your change is fully reviewed and
passes CI. You may click it to merge your PR. If you do not have write access,
or if you believe that status checks are failing for inconsequential reasons,
ask your platform reviewer to perform the merge for you.


Continuous Integration Notes
============================
.. toctree::
    :maxdepth: 1

    CDash <https://drake-cdash.csail.mit.edu/index.php?project=Drake>
    jenkins

IDE and Text Editor Notes
=========================

.. toctree::
    :maxdepth: 1

    clion
    `Eclipse <https://github.com/tkoolen/drake/wiki/Eclipse-setup-(experimental)>`_


Sublime Text
------------

Recommended packages to install:

1. https://packagecontrol.io/packages/SublimeLinter-cpplint

To display the current file's full path in the title bar on OSX, open your user preferences by going to "Sublime Text," "Preferences," "Settings - User." Then add the following to your user preferences:

    "show_full_path": true



Operating System Notes
======================
.. toctree::
    :maxdepth: 1

    development_on_osx

Programming Style Notes
=======================
.. toctree::
    :maxdepth: 1

    code_style_guide
    code_style_tools

Version Control
===============
.. toctree::
    :maxdepth: 1

    no_push_to_origin
