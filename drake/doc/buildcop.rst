*********
Build Cop
*********

.. _overview:

Overview
--------

The Drake build cop monitors `post-submit <https://drake-
jenkins.csail.mit.edu/view/Continuous/>`_ and `nightly <https://drake-
jenkins.csail.mit.edu/view/Nightly/>`_ continuous integration failures in the
RobotLocomotion/drake GitHub repo.

The build cop will rotate on a weekly basis, initially through members of the
Kitware team, but may extend to include members of the TRI team in future. The 
`schedule <https://github.com/RobotLocomotion/drake-ci/wiki/Build-Cop-Rotation>`_
is maintained on the RobotLocomotion/drake-ci wiki.

.. _process:

Process
-------
The build cop is expected to be on duty during normal business hours Eastern
Time, approximately 9am to 5pm on weekdays, holidays excepted. Developers are
encouraged, but not required, to merge pull requests during times when the build
cop is on duty. Nightly build failures will be addressed the following weekday
morning.

When a CI build failure occurs, the build cop will be notified by email.
Notifications are sent to ``drake-alerts+jenkins@tri.global`` and
``drake-developers+build-cop@kitware.com``. The build cop will triage the
failure by identifying the recent commits or other problem that caused the CI
build to fail.

In the case of an issue with the CI infrastructure, Kitware will be responsible
for fixing the issue.

In the case of an issue with recent commits, the build cop will contact the
authors of the commits and prepare a new pull request that reverts those
commits. If the authors are not responsive within 15 minutes, or are unable to
fix the failure within 60 minutes, the build cop will merge the pull request to
revert the commits and verify that the continuous builds triggered by that merge
pass.

.. _revert_template:

Revert Template
---------------
When creating a revert PR, the build cop will assign that PR to the original
author, and include the following template in the PR description.

::

 Dear $AUTHOR,

 The oncall build cop, $BUILD_COP, believes that your PR $NUMBER may have broken
 Drake's continuous integration build [1]. It is possible to break the build
 even if your PR passed continuous integration on presubmit, because additional
 platforms and tests are built in postsubmit.

 The specific build failures under investigation are:
 $LINK_TO_BROKEN_BUILD_ON_JENKINS
 $LINK_TO_BROKEN_BUILD_ON_JENKINS

 Therefore, the build cop has created this revert PR and started a complete
 postsubmit build to determine whether your PR was in fact the cause of the
 problem. If that build passes, this revert PR will be merged 60 minutes from
 now. You can then fix the problem at your leisure, and send a new PR to
 reinstate your change.

 If you believe your original PR did not actually break the build, please
 explain on this thread.

 If you believe you can fix the break promptly in lieu of a revert, please
 explain on this thread, and send a PR to the build cop for review ASAP.

 If you believe your original PR definitely did break the build and should be
 reverted, please review and LGTM this PR. This allows the build cop to merge
 without waiting for CI results.

 For advice on how to handle a build cop revert, see [2].

 Thanks!
 Your Friendly Oncall Buildcop

 [1] CI Dashboard: https://drake-jenkins.csail.mit.edu/view/Continuous/
 [2] http://drake.mit.edu/buildcop.html#workflow-for-handling-a-build-cop-revert

.. _handling_a_build_cop_revert:

Workflow for Handling a Build Cop Revert
----------------------------------------

Suppose your merged PR was reverted on the master branch. What do you do?

Here's one workflow:

1. Create a new development branch based off of the ``HEAD`` of master.

2. `Revert <https://git-scm.com/docs/git-revert>`_ the revert of your
   originally-merged PR to get your changes back.

3. Debug the problem. This may require you to
   :ref:`run on-demand continuous integration builds <run_specific_build>` to
   ensure the problem that caused your PR to be reverted was actually fixed.

4. Commit your changes into your new branch.

5. Issue a new PR containing your fixes. Be sure to link to the build cop revert
   PR in your new PR.
