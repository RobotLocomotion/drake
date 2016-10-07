**********************************
GitHub PR Interaction with Jenkins
**********************************

When a new pull request is opened in the project and the author of the pull
request is not a member of the GitHub project or whitelisted, the Jenkins
GitHub Pull Request Builder @drake-jenkins-bot will ask
"Can one of the admins verify this patch?"

Respond:

* "@drake-jenkins-bot ok to test" to accept this pull request for testing.
* "@drake-jenkins-bot test this please" for a one time test run.
* "@drake-jenkins-bot add to whitelist" to add the author to the whitelist.

If the build fails for other various reasons you can rebuild:

* "@drake-jenkins-bot retest this please" to start a new build.

You can also view the `Jenkins UI <https://drake-jenkins.csail.mit.edu/>`_
directly.

.. _run_specific_build:

Running an On-Demand Build
==========================

There are a number of Jenkins builds that do not normally run pre-merge, but
do run post-merge or nightly.  The post-merge and nightly builds include
long-running tests, lower-priority platforms (e.g. Mac), and
specialized options (e.g.
`MemorySanitizer <https://github.com/google/sanitizers/wiki/MemorySanitizer>`_).
Members of the RobotLocomotion organization can manually schedule these builds
on pull requests that have not yet been merged, or on arbitrary commits in the
``RobotLocomotion/drake`` repository.

1. Log in to `Jenkins <https://drake-jenkins.csail.mit.edu/>`_ using GitHub OAuth.
2. Go to the `list of Experimental builds <https://drake-jenkins.csail.mit.edu/view/Experimental/>`_.
3. Click on the specific build you want to schedule.
4. Click on "Build with Parameters" in the left menu.
5. Check the ``debug`` box if you want to build in Debug configuration.
6. Enter ``pr/XYZ/head`` (HEAD of pull request), ``pr/XYZ/merge`` (pull request merged with master), or the desired commit SHA in the ``sha1`` field.
7. Click ``Build``.

The list of Experimental builds includes a build named ``experimental``, which
automatically runs pre-merge, as well as numerous other builds for on-demand
use. To help identify the on-demand build you want to run, you can consult the
`list of Continuous builds <https://drake-jenkins.csail.mit.edu/view/Continuous/>`_
and the
`list of Nightly builds <https://drake-jenkins.csail.mit.edu/view/Nightly/>`_,
but you can't schedule Continuous or Nightly builds directly.
