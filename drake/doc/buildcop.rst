*********
Build Cop
*********

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
