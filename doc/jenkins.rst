**********************************
GitHub PR Interaction with Jenkins
**********************************

When a new pull request is opened in the project and the author of the pull
request is not a member of the RobotLocomotion GitHub organization, the Jenkins
GitHub Pull Request Builder @drake-jenkins-bot will not automatically schedule
builds.

To allow the pull request to be tested, a member of the RobotLocomotion
organization may comment:

* ``@drake-jenkins-bot ok to test`` to accept this pull request for testing.
* ``@drake-jenkins-bot test this please`` for a one time test run.

If the build fails for other various reasons you can rebuild:

* ``@drake-jenkins-bot retest this please`` to start a new build.

You can also view the `Jenkins UI <https://drake-jenkins.csail.mit.edu/>`_
directly.

.. _run_specific_build:

Running an On-Demand Build
==========================

There are a number of Jenkins builds that do not normally run pre-merge, but do
run post-merge, nightly, or weekly. These builds include lower-priority
platforms (e.g., macOS), and specialized options (e.g.,
`UndefinedBehaviorSanitizer <https://releases.llvm.org/6.0.0/tools/clang/docs/UndefinedBehaviorSanitizer.html>`_).
Members of the RobotLocomotion organization can manually schedule these builds
on pull requests that have not yet been merged, or on arbitrary commits in the
``RobotLocomotion/drake`` repository.

To schedule a build of an open pull request merged with master, comment:

* ``@drake-jenkins-bot <job-name> please``


where ``<job-name>`` is the name of an
`experimental job <https://drake-jenkins.csail.mit.edu/view/Experimental/>`_.

For example:

* ``@drake-jenkins-bot mac-high-sierra-clang-bazel-experimental-release please``
* ``@drake-jenkins-bot linux-bionic-clang-bazel-experimental-valgrind-memcheck please``

Alternatively, to schedule a build of an open pull request or arbitrary commit
in the ``RobotLocomotion/drake`` repository:

1. **Log in** to `Jenkins <https://drake-jenkins.csail.mit.edu/>`_ using GitHub OAuth.
   (Make sure that you see your name the upper-right corner, *not* the words "Log in".)
2. Go to the `list of experimental builds <https://drake-jenkins.csail.mit.edu/view/Experimental/>`_.
3. Click on the specific build you want to schedule.
4. Click on "Build with Parameters" in the left menu.
5. Enter ``pr/XYZ/head`` (HEAD of pull request), ``pr/XYZ/merge`` (pull request
   merged with master), or the desired commit SHA in the ``sha1`` field.
6. Click ``Build``.

The list of experimental builds includes builds that automatically run on opened
and updated pull requests, as well as numerous other builds for on-demand use.
To help identify the on-demand build you want to run, you can consult the lists
of `continuous <https://drake-jenkins.csail.mit.edu/view/Continuous/>`_,
`nightly <https://drake-jenkins.csail.mit.edu/view/Nightly/>`_, and
`weekly <https://drake-jenkins.csail.mit.edu/view/Weekly/>`_ builds,
but you should not schedule continuous, nightly, or weekly builds directly.
