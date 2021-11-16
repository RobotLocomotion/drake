---
title: Build Cop
---

# Overview

The Drake build cop monitors [continuous](https://drake-jenkins.csail.mit.edu/view/Continuous%20Production/),
[nightly](https://drake-jenkins.csail.mit.edu/view/Nightly%20Production/), and
[weekly](https://drake-jenkins.csail.mit.edu/view/Weekly%20Production/)
production continuous integration failures in the
[RobotLocomotion/drake](https://github.com/RobotLocomotion/drake) GitHub
repo.

The build cop will rotate on a weekly basis. The
[schedule](https://github.com/RobotLocomotion/drake-ci/wiki/Build-Cop-Rotation)
is maintained on the
[RobotLocomotion/drake-ci](https://github.com/RobotLocomotion/drake-ci) wiki.

# Process

Two build cops are expected to be on duty on weekdays, holidays excepted. At
least one build cop should be on duty during normal business hours Eastern Time,
approximately 9am to 5pm. Developers are encouraged, but not required, to merge
pull requests during times when the build cop is on duty. Nightly and weekly
build failures will be addressed the following weekday morning.

When a CI build failure occurs, the build cop will be notified by email.
Notifications are sent to ``drake-alerts+jenkins@tri.global`` and
``drake-developers+build-cop@kitware.com``. To receive these notifications,
subscribe to the ``drake-alerts`` Google Group at TRI or the
``drake-developers`` Google Group at Kitware. The build cop will triage the
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

Use the [DrakeDevelopers Slack channel #buildcop](https://drakedevelopers.slack.com/messages/buildcop/details/)
to discuss build issues with your partner build cop and other Drake
contributors.

At the end of each rotation, the build cop should complete the
[build cop review and retrospective](https://docs.google.com/document/d/120AOAaamIMO-SM1UaJ6vfzpA15LnXHexDF4a7MLAS3o/edit#heading=h.sxk1djc2v0yg),
and should notify the next build cop on the [DrakeDevelopers Slack channel #buildcop](https://drakedevelopers.slack.com/messages/buildcop/details/).

# Revert Template

When creating a revert PR, the build cop will assign that PR to the original
author, and include the following template in the PR description.

```

 Dear $AUTHOR,

 The on-call build cop, $BUILD_COP, believes that your PR $NUMBER may have
 broken one or more of Drake's continuous integration builds [1]. It is
 possible to break a build even if your PR passed continuous integration
 pre-merge because additional platforms are tested post-merge.

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
 Your Friendly On-call Build Cop

 [1] CI Production Dashboard: https://drake-jenkins.csail.mit.edu/view/Production/
 [2] https://drake.mit.edu/buildcop.html#workflow-for-handling-a-build-cop-revert
```

# Workflow for Handling a Build Cop Revert

Suppose your merged PR was reverted on the master branch. What do you do?

Here's one workflow:

1. Create a new development branch based off of the ``HEAD`` of master.
2. [Revert](https://git-scm.com/docs/git-revert) the revert of your
   originally-merged PR to get your changes back.
3. Debug the problem. This may require you to
   [run on-demand continuous integration](/jenkins.html#scheduling-an-on-demand-build) to
   ensure the problem that caused your PR to be reverted was actually fixed.
4. Commit your changes into your new branch.
5. Issue a new PR containing your fixes. Be sure to link to the build cop revert
   PR in your new PR.


# Build Cop Playbook

This section is a quick-reference manual for the on-call build cop.

## Monitor the Build

Check the [Continuous Production](https://drake-jenkins.csail.mit.edu/view/Continuous%20Production/)
build dashboard in Jenkins at least once an hour during on-call hours. These
builds run after every merge to Drake. Also check the
[Nightly Production](https://drake-jenkins.csail.mit.edu/view/Nightly%20Production/)
build dashboard every morning and
[Weekly Production](https://drake-jenkins.csail.mit.edu/view/Weekly%20Production/)
build dashboard on Monday morning. These builds are unusually
resource-intensive, and therefore run at most once per day.

If any Continuous, Nightly, or Weekly Production builds turn yellow or red, you
need to act.

In Jenkins, builds that are in progress (blinking on and off) will show the
color of the previous build.

Note that CDash pages may take a minute to populate.

## Respond to Breakage

There are various reasons the build might break. Diagnose the failure, and
then take appropriate action. This section lists some common failures and
recommended responses. However, build cops often have to address unexpected
circumstances. Do not expect this list to be complete, and always apply your
own judgment.

In almost any build breakage, the first information-gathering step is to
click on the build that is yellow or red in Jenkins, then click on the first
breaking change in the Build History. You will see a list of the new commits
in that particular run.

Determine if an open GitHub Drake issue describes the situation. For example,
some tests are flaky for reasons that have no known resolution, but are
described by Drake issues. If you find that your broken build is described by
such an issue, consider adding the build information to the issue for future
analysis. The [build cop review and retrospective](https://docs.google.com/document/d/120AOAaamIMO-SM1UaJ6vfzpA15LnXHexDF4a7MLAS3o/edit#heading=h.sxk1djc2v0yg)
also describes current build issues.

## Broken Compile or Test

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
[template message](/buildcop.html#revert-template) to communicate  with the author, and
proceed as specified in that message.

[Manually schedule](/jenkins.html#run-specific-build) the failing build as an
experimental build on the rollback PR. If it passes, the odds are good that you
have found the culprit. Proceed as specified in the template message.

## Flaky Test

Sometimes people introduce code that makes a test non-deterministic, failing
on some runs and passing on others. You cannot reliably attribute a flaky test
failure to the first failing build, because it may have passed by chance for
the first few continuous builds after the culprit PR landed.

Test failures will be yellow in Jenkins. If the list of commits in the breaking
change does not include any plausible culprits, you may be looking at a flaky
test.  Look through earlier commits one-by-one for plausible culprits.
After you identify one, create a rollback by clicking "Revert" in the
GitHub UI. Use the [template message](/buildcop.html#revert-template) to communicate
with the author, and proceed as specified in that message.

## Restarting Mac Nightly Builds

Occasionally there will be flaky tests or timeouts in the Mac nightly builds.
While it is tempting to restart these builds to clear the errors, Mac resources
are limited and restarting the long-running nightly builds may tie up resources
needed for continuous builds. In addition, too many simultaneous Mac builds
will increase the chances of timeouts and other flakes. Build cops should use
their best judgement, keeping in mind the following guidelines:

* If the nightly job is mirrored by a continuous job, don't re-run.
* If the test passed last build, don't re-run.
* If it is a linter only timeout, don't re-run.
* If there are many timeouts, you may consider re-running.
* If the timed-out test failed last build (not just timed out), you may consider re-running.


## Broken CI Script

Sometimes people merge changes to the Drake CI scripts that result in spurious
CI failures. The list of commits in Jenkins for each continuous build includes
the [drake-ci](https://github.com/RobotLocomotion/drake-ci) repository as well
as Drake proper. Consider whether those changes are possible culprits.

If you believe a CI script change is the culprit, contact the author.
If they are not responsive, revert the commit yourself and see what happens on
the next continuous build. There are no pre-merge builds you can run that
exercise changes to the CI scripts themselves.

## Infrastructure Flake

The machinery of the CI system itself sometimes fails for reasons unrelated to
any code change. The most common infrastructure flakes include:

* Unable to obtain a Gurobi license.
* Broken connection to a Mac build agent.

Infrastructure flakes will be red in Jenkins. If you believe you are looking at
an infrastructure flake, run the build manually at HEAD. If it passes, you are
definitely looking at an infrastructure flake, and no further action is
required. If you believe the rate of a particular infrastructure flake has
increased, alert Kitware by assigning a GitHub issue to `@BetsyMcPhail`.

Note that "slow read" warnings during Bazel builds are due to the relative
slowness of the remote storage used by the CI infrastructure when compared to
storage connected to the local bus on a local developer workstation build and
can be safely ignored.

If you see "All nodes of label <label> are offline", this should disappear
eventually and the build should run, once Jenkins gets a node booted up.

## Infrastructure Collapse

Occasionally, some piece of CI infrastructure completely stops working. For
instance, GitHub, AWS, or MacStadium could have an outage, or our Jenkins server
could crash or become wedged.  During infrastructure collapses, lots of builds
will turn red and stay red.

Attempt to figure out what infrastructure collapsed. If it's under our control,
alert Kitware by assigning a GitHub issue to `@BetsyMcPhail`. If it's under a
vendor's control, spread the news and simply wait it out.

## Drake External Examples

Details of failures in the [drake-external-examples](https://github.com/RobotLocomotion/drake-external-examples/)
repository, which may be denoted by red "build failing" icons at the top of the build
dashboard on Jenkins, should be posted to the [#buildcop](https://drakedevelopers.slack.com/messages/buildcop/details/)
channel on Slack, ensuring that `@betsymcphail` is mentioned in the message.
