**********************************
GitHub PR Interaction with Jenkins
**********************************

When a new pull request is opened in the project and the author of the pull request is not a member of the GitHub project or whitelisted, the Jenkins GitHub Pull Request Builder will ask "Can one of the admins verify this patch?"
Respond:

* "ok to test" to accept this pull request for testing.
* "test this please" for a one time test run.
* "add to whitelist" to add the author to the whitelist.

If the build fails for other various reasons you can rebuild:
* "retest this please" to start a new build.

You can also view the `Jenkins interface <https://drake-jenkins.csail.mit.edu/>`_ directly.
