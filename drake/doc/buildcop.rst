.. _build_cop:

*********
Build Cop
*********

.. _overview:

Overview
--------

The Drake build cop monitors `continuous production <https://drake-
jenkins.csail.mit.edu/view/Continuous%20Production/>`_ and `nightly production
<https://drake-jenkins.csail.mit.edu/view/Nightly%20Production/>`_ continuous
integration failures in the RobotLocomotion/drake GitHub repo.

The build cop will rotate on a weekly basis. The
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

 The on-call build cop, $BUILD_COP, believes that your PR $NUMBER may have broken
 Drake's continuous integration build [1]. It is possible to break the build
 even if your PR passed continuous integration pre-merge, because additional
 platforms and tests are built post-merge.

 The specific build failures under investigation are:
 $LINK_TO_BROKEN_BUILD_ON_JENKINS
 $LINK_TO_BROKEN_BUILD_ON_JENKINS

 Therefore, the build cop has created this revert PR and started a complete
 post-merge build to determine whether your PR was in fact the cause of the
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


.. _build_cop_playbook:

Build Cop Playbook
------------------
This section is a quick-reference manual for the on-call build cop.

Monitor the Build
^^^^^^^^^^^^^^^^^
Check the `Continuous <https://drake-jenkins.csail.mit.edu/view/Continuous/>`_
build dashboard in Jenkins at least once an hour during on-call hours. These
builds run after every merge to Drake. Also check the
`Nightly Production <https://drake-jenkins.csail.mit.edu/view/Nightly%20Production/>`_
build dashboard every morning. These builds are unusually resource-intensive,
and therefore run just once per day.

If any Continuous or Nightly Production builds turn yellow or red, you need
to act.

Respond to Breakage
^^^^^^^^^^^^^^^^^^^
There are various reasons the build might break. Diagnose the failure, and
then take appropriate action. This section lists some common failures and
recommended responses. However, build cops often have to address unexpected
circumstances. Do not expect this list to be complete, and always apply your
own judgment.

In almost any build breakage, the first information-gathering step is to
click on the build that is yellow or red in Jenkins, then click on the first
breaking change in the Build History. You will see a list of the new commits
in that particular run.

Broken Compile or Test
**********************
Sometimes people merge code that doesn't compile, or that fails a test.
This can happen for several reasons:

* The platform or test case only runs post-merge.
* An administrator performed an override-merge of the culprit PR,
  circumventing pre-merge checks.
* The failure is an interaction between the culprit PR and some other
  recent change to master.

Compile failures will be red in Jenkins. Test failures will be yellow.
Consult the list of commits in the breaking change to identify possible culprit
PRs. Try to rule out some of those PRs by comparing their contents to the
specifics of the failure. For any PRs you cannot rule out, create a rollback
by clicking "Revert" in the GitHub UI. Use the
:ref:`template message <revert_template>` to communicate  with the author, and
proceed as specified in that message.

:ref:`Manually schedule <run_specific_build>` the failing build as an
experimental build on the rollback PR. If it passes, the odds are good that you
have found the culprit. Proceed as specified in the template message.

Flaky Test
**********
Sometimes people introduce code that makes a test non-deterministic, failing
on some runs and passing on others. You cannot reliably attribute a flaky test
failure to the first failing build, because it may have passed by chance for
the first few continuous builds after the culprit PR landed.

Test failures will be yellow in Jenkins. If the list of commits in the breaking
change does not include any plausible culprits, you may be looking at a flaky
test.  Look through earlier commits one-by-one for plausible culprits.
After you identify one, create a rollback by clicking "Revert" in the
GitHub UI. Use the :ref:`template message <revert_template>` to communicate
with the author, and proceed as specified in that message.

Broken CI Script
****************
Sometimes people merge changes to the Drake CI scripts that result in spurious
CI failures. The list of commits in Jenkins for each continuous build includes
the `drake-ci <https://github.com/RobotLocomotion/drake-ci>`_ repository as well
as Drake proper. Consider whether those changes are possible culprits.

If you believe a CI script change is the culprit, contact the author.
If they are not responsive, revert the commit yourself and see what happens on
the next continuous build. There are no pre-merge builds you can run that
exercise changes to the CI scripts themselves.

Infrastructure Flake
********************
The machinery of the CI system itself sometimes fails for reasons unrelated to
any code change. The most common infrastructure flakes include:

* Unable to obtain a MATLAB license.
* Broken connection to a Mac build agent.

Infrastructure flakes will be red in Jenkins. If you believe you are looking at
an infrastructure flake, run the build manually at HEAD. If it passes, you are
definitely looking at an infrastructure flake, and no further action is
required. If you believe the rate of a particular infrastructure flake has
increased, alert Kitware by assigning a GitHub issue to both @BetsyMcPhail and
@jamiesnape.

Infrastructure Collapse
***********************
Occasionally, some piece of CI infrastructure completely stops working. For
instance, GitHub, AWS, or MacStadium could have an outage, or our Jenkins server
could crash or become wedged.  During infrastructure collapses, lots of builds
will turn red and stay red.

Attempt to figure out what infrastructure collapsed. If it's under our control,
alert Kitware by assigning a GitHub issue to both @BetsyMcPhail and
@jamiesnape. If it's under a vendor's control, spread the news and simply wait
it out.
