.. _platform_reviewer_checklists:

****************************
Platform Reviewer Checklists
****************************

There are several policies documented in :ref:`GitHub Issue Management
<issues>` that the on-call platform reviewer should enforce.  The on-call
reviewer should run through this checklist at least once per day.

Search for `issues without an assigned team
<https://github.com/RobotLocomotion/drake/issues?utf8=%E2%9C%93&q=is%3Aissue+is%3Aopen+-label%3A%22team%3A+dynamics%22+-label%3A%22team%3A+kitware%22+-label%3A%22team%3A+manipulation%22+-label%3A%22team%3A+russ%22+-label%3A%22team%3A+robot+locomotion+group%22>`_
and assign a team.  When in doubt, seek advice on slack.

Search for `issues without an assigned individual
<https://github.com/RobotLocomotion/drake/issues?q=is%3Aissue+is%3Aopen+no%3Aassignee>`_
and assign an owner.  When in doubt, assign :ref:`the lead <issues-team>`
associated with the issue's ``team`` label.

Search for `pull requests that need review
<https://github.com/RobotLocomotion/drake/pulls?q=is%3Aopen+is%3Apr+no%3Aassignee+-label%3A%22status%3A+do+not+review%22>`_
and (probably) assign a feature reviewer.  This is intended to make sure that
requests from outside developers receive timely attention.  For a pull request
by a core Drake Developer, leaving it unassigned may be acceptable when it is
clearly an early work-in-progress -- but if it is unassigned for several days,
you should probably encourage the developer to label it "do not review" for
clarity.
