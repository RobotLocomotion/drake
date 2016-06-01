**********************************
GitHub PR Interaction with Jenkins
**********************************

When a new pull request is opened in the project and the author of the pull
request is not a member of the GitHub project or whitelisted, the Jenkins
GitHub Pull Request Builder will ask "Can one of the admins verify this patch?"

Respond:

* "ok to test" to accept this pull request for testing.
* "test this please" for a one time test run.
* "add to whitelist" to add the author to the whitelist.

If the build fails for other various reasons you can rebuild:
* "retest this please" to start a new build.

You can also view the `Jenkins UI <https://drake-jenkins.csail.mit.edu/>`_
directly.

.. _run_specific_build:

Running a Specific Build Before Merge
=====================================

There are a number of Jenkins builds that do not normally run pre-merge, but
do run post-merge or nightly.  The post-merge and nightly builds include
long-running tests, lower-priority platforms (e.g. Windows MATLAB), and
specialized options (e.g. msan). Members of the RobotLocomotion organization
can manually trigger these builds on pull requests that have not yet been
merged.

1. Log in to Jenkins using GitHub OAuth.
2. Go to the `list of Experimental builds <https://drake-jenkins.csail.mit.edu/view/Experimental/>`_.
3. Click on the specific build you want to trigger.
4. Click on "Build with Parameters" in the left menu.
5. Check the ``debug`` box if you want to build in Debug configuration.
6. Enter ``pr/XYZ`` in the ``sha1`` field.
7. Click ``Build``.

The list of Experimental builds includes a build named ``experimental``, which
runs pre-merge, and numerous other builds for one-off use. To help identify the
Experimental build you want to run, you can consult the
`list of Continuous builds <https://drake-jenkins.csail.mit.edu/view/Continuous/>`_
and the
`list of Nightly builds <https://drake-jenkins.csail.mit.edu/view/Nightly/>`_,
but you can't trigger those builds directly.
