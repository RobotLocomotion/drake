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

Running a Specific Build Before Submitting
==========================================

There are a number of Jenkins builds that do not normally run pre-submit, but
do run post-submit or nightly.  The post-submit and nightly builds include
long-running tests, lower-priority platforms (e.g. Windows MATLAB), and
specialized options (e.g. msan).  Users with login access to Jenkins can
manually trigger these builds on pull requests that have not yet been merged.

1. Log in to Jenkins using GitHub OAuth.
2. Go to the `list of Experimental builds <https://drake-jenkins.csail.mit.edu/view/Experimental/>`_.

   a. In this list, the build named "experimental" runs on presubmit.
   b. The other specific builds are for one-off use.

3. Click on the specific build you want to trigger.
4. Click on "Build with Parameters" in the left menu.
5. Check the ``debug`` box if you want to build in Debug configuration.
6. Enter ``pr/XYZ`` in the ``sha1`` field.
7. Click ``Build``.
