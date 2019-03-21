.. _developer_notes:

**************
For Developers
**************

.. contents:: `Table of contents`
   :depth: 3
   :local:

.. _pull_request:

Introduction
============

If you have improvements to Drake, send us your pull requests!

Our standard workflow is to fork `Drake's official Github repository
<https://github.com/RobotLocomotion/drake/>`_ into your
own Github account and then push your changes into a branch on your fork. Once
you believe your code is ready to be merged into Drake's primary repository,
open a `pull request <https://help.github.com/articles/using-pull-requests/>`_
via the Github website. Your code will then undergo an interactive review
process and :ref:`Continuous Integration (CI) <continuous_integration_notes>`
tests before it is merged into
`Drake's primary repository <https://github.com/RobotLocomotion/drake>`_.

Drake's :ref:`CI service <continuous_integration_notes>` runs on all pull requests each time they are
submitted and updated. Pull requests cannot be merged into master unless all
unit tests pass on all
:ref:`supported platform configurations <supported-configurations>`.

Drake's CI server also runs continuously on
`Drake's primary master branch <https://github.com/RobotLocomotion/drake>`_
using an even more comprehensive set of unit tests.
If problems are detected on this branch, the build cop will
:ref:`revert the PRs that most likely caused the problem <build_cop>`.
To increase the likelihood that your pull requests pass CI tests and are not
reverted, you can run the unit tests locally. Instructions for how to do that
are provided :ref:`here <unit-test-instructions>`. Note, however, that there are
many computationally-demanding tests and running the entire test suite can take
several hours depending on your machine.

We would like to hear about your success stories if you've used Drake in your
own projects.  Please consider contributing to our :doc:`gallery` by editing
``doc/gallery.rst`` and submitting a pull request with the update!

Licensing
=========

**Important note:** Drake is an open source project licensed under
extremely flexible terms intended to encourage use by anyone, for any
purpose. When you make a contribution to the Drake project, you are
agreeing to do so under those same terms.

Testing
=======

.. toctree::
    :maxdepth: 1

    unit_testing_instructions
    downstream_testing

.. See :doc:`unit_testing_instructions`

.. _supported-configurations:

Supported Configurations
========================

The following table shows the configurations and platforms that Drake
officially supports. Supported configurations are tested in continuous
integration. All other configurations are provided on a best-effort basis.

Drake requires a compiler running in C++14 mode or greater. MATLAB is only
supported for CMake builds using the "Unix Makefiles" generator.

+----------------------------------+---------------+-----------------+------------+-------------------+--------+
| Operating System                 | Build System  | C/C++ Compiler  | Java       | MATLAB (Optional) | Python |
+==================================+===============+=================+============+===================+========+
+----------------------------------+---------------+-----------------+------------+-------------------+--------+
| Ubuntu 16.04 LTS (Xenial Xerus)  | | Bazel 0.23  | | Clang 6.0     | OpenJDK 8  | R2017a            | 2.7    |
|                                  | | CMake 3.5   | | GCC 5.4       |            |                   |        |
+----------------------------------+---------------+-----------------+------------+-------------------+--------+
| Ubuntu 18.04 LTS (Bionic Beaver) | | Bazel 0.23  | | Clang 6.0     | OpenJDK 11 | R2018b            | 2.7    |
|                                  | | CMake 3.10  | | GCC 7.3       |            |                   | 3.6    |
+----------------------------------+---------------+-----------------+------------+                   +--------+
| macOS High Sierra (10.13)        | | Bazel 0.23  | Apple LLVM 10.0 | Oracle 11  |                   | 2.7    |
+----------------------------------+ | CMake 3.13  |                 |            |                   | 3.7    |
| macOS Mojave (10.14)             |               |                 |            |                   |        |
+----------------------------------+---------------+-----------------+------------+-------------------+--------+


CPython is the only Python implementation supported. On all platforms, Python 2
is the default version. On Ubuntu, amd64 (i.e., x86_64) is the only supported
architecture.

The following configurations are presently untested in continuous integration:

-   macOS Mojave: C++, Python
-   macOS, Ubuntu Bionic: MATLAB

.. _configuration-management-non-determinism:

Configuration Management Non-Determinism
----------------------------------------

The indicated versions for build systems and languages are recorded after
having been tested on Continuous Integration.

Due to how the Debian ``apt`` and Homebrew package managers work, you may not
have these exact versions on your system when (re)running
``install_prereqs.sh``. In general, later minor versions for more stable
packages (e.g. CMake, compilers) should not prove to be too much of an issue.

For less stable packages, such as Bazel, later minor versions may cause
breakages. If you are on Ubuntu, please rerun ``install_prereqs.sh`` as it can
downgrade Bazel. If on Mac, there is no easy mechanism to downgrade with
Homebrew; however, we generally try to stay on top of Bazel versions.

If you have tried and are unable to configure your system by
:ref:`following the instructions <build_from_source>`, please do not hesitate
to :ref:`ask for help <getting_help>`.

.. _binary-packages:

Binary Packages
---------------

The binary releases of Drake are built with GCC 5.4 on Ubuntu Xenial, GCC 7.3
on Ubuntu Bionic, and Apple LLVM 10.0 on macOS High Sierra.

The links for these packages are listed in :ref:`binary-installation`.

Issue Tracking
==============

.. toctree::
    :maxdepth: 1

    issues
    platform_reviewer_checklist

Code Review
===========

Review Process
--------------

For complex changes, especially those that will span multiple PRs, please
open a GitHub issue and solicit design feedback before you invest a lot of
time in code.

Before your submit a pull request, please consult the
:ref:`Code Review Checklist <code-review-checklist>`,
where a list of the most frequent problems are collected.

Be prepared to engage in active code review on your pull requests.  The Drake
code review process has two phases: feature review and platform review. You
are responsible for finding reviewers, and for providing them the information
they need to review your change effectively. If a reviewer asks you for more
information, that is a sign you should add more documentation to your PR.

A PR generally *should not* include more than 750 added or changed lines (the
green ``+###`` number as reported by github), and *must not* include more than
1500 lines, with the following exemptions:

  - Data files do not count towards the line limit.

  - Machine-generated changes do not count towards the line limit.

  - Files in
    :ref:`Special Directories <directory_structure_special_directories>`
    do not count towards the line limit.

  - This rule may be overridden by agreement of at least two platform reviewers
    (listed below).

The utility ``tools/prstat`` will report the total added or changed
lines, excluding files that are easily identified to meet the exemptions above.

We use https://reviewable.io for code reviews. You can sign in for free with
your GitHub identity. Before your first code review, please take a look at
:doc:`reviewable`.

If you have an expected pace for your review, please add a ``priority`` label
(which have different meanings for PRs and
:ref:`for issues <issues-priority>`). The response expectations, for both the
author and reviewer:

- ``priority: emergency`` - Very quick response time, nominally reserved for
  build cop.
- ``priority: high`` - Some urgency, quick response time.
- ``priority: medium`` - (Default) Normal response time.
- ``priority: low`` - No rush.
- ``priority: backlog`` - Give priority to all other PRs on your plate.

If you are an external contributor, you will need to request that a priority be
added by a Drake Developer.

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

- @EricCousineau-TRI (Toyota Research Institute)
- @ggould-tri (Toyota Research Institute)
- @jwnimmer-tri (Toyota Research Institute)
- @sammy-tri (Toyota Research Institute)
- @SeanCurtis-TRI (Toyota Research Institute)
- @sherm1 (Toyota Research Institute)
- @soonho-tri (Toyota Research Institute)
- @RussTedrake (MIT / Toyota Research Institute)

**Merge.** If you have write access to RobotLocomotion/drake, a green
"Merge Pull Request" button will appear when your change is fully reviewed and
passes CI. You may click it to merge your PR. If you do not have write access,
or if you believe that status checks are failing for inconsequential reasons,
ask your platform reviewer to perform the merge for you.

**After Merge.** If your PR breaks continuous integration, the :doc:`buildcop`
will contact you to work out a resolution.

Review Process Tooling
----------------------

.. toctree::
    :maxdepth: 1

    reviewable

.. _continuous_integration_notes:

User Assistance
===============

The user-facing instructions for requesting assistance are located in
:ref:`getting_help`. The two main options for requesting assistance are either
posting a GitHub issue or a StackOverflow question.

Handling User GitHub Issues
---------------------------

See :ref:`issues`.

If a GitHub issue should instead be a StackOverflow question (e.g. it is of a
tutorial nature that does not require code or documentation modification),
please request that the user repost the question on StackOverflow, post the
new link on the GitHub issue, and close the issue.

Handling User StackOverflow Questions
-------------------------------------

Please subscribe to the ``drake`` tag by following
`these general instructions <https://meta.stackoverflow.com/a/336515/7829525>`_,
if you are able to.

Please also monitor for `unanswered StackOverflow posts
<https://stackoverflow.com/unanswered/tagged/drake?tab=noanswers>`_
once per day. If there are unanswered questions that you are unsure of the
answer, consider posting on the Slack ``#onramp`` channel to see if someone
can can look into the question.

The following developers are subscribed to the ``drake`` tag, and will monitor
it:

  - Russ Tedrake
  - Eric Cousineau

Continuous Integration Notes
============================
.. toctree::
    :maxdepth: 1

    CDash <https://drake-cdash.csail.mit.edu/index.php?project=Drake>
    jenkins
    buildcop

Documentation Instructions
==========================
.. toctree::
    :maxdepth: 1

    documentation_instructions
    doxygen_instructions
    sphinx_instructions

IDE and Text Editor Notes
=========================

.. toctree::
    :maxdepth: 1

    clion
    Eclipse <https://github.com/tkoolen/drake/wiki/Eclipse-setup-(experimental)>
    sublime_text
    unicode_tips_tricks
    vim

Operating System Notes
======================
.. toctree::
    :maxdepth: 1

    development_on_mac

Programming Style Notes
=======================
.. toctree::
    :maxdepth: 1

    code_review_checklist
    code_style_guide
    code_style_tools
    directory_structure

Version Control
===============
.. toctree::
    :maxdepth: 1

    no_push_to_origin

