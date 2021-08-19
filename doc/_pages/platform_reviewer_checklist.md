---
title: Platform Reviewer Checklists
---

There are several policies documented in [GitHub Issue Management](/issues.html)
that the on-call platform reviewer should enforce.  The on-call
reviewer should run through this checklist at least once per day.

Search for [issues without an assigned team](https://github.com/RobotLocomotion/drake/issues?utf8=%E2%9C%93&q=is%3Aissue+is%3Aopen+-label%3A%22team%3A+dynamics%22+-label%3A%22team%3A+kitware%22+-label%3A%22team%3A+manipulation%22+-label%3A%22team%3A+robot+locomotion+group%22)
and assign a team.  When in doubt, seek advice on slack.

Search for [issues without an assigned individual](https://github.com/RobotLocomotion/drake/issues?q=is%3Aissue+is%3Aopen+no%3Aassignee)
and assign an owner.  When in doubt, assign [the lead](/issues.html#team)
associated with the issue's ``team`` label.

Search for [pull requests with no assignee](https://github.com/RobotLocomotion/drake/pulls?q=is%3Aopen+is%3Apr+no%3Aassignee+-label%3A%22status%3A+do+not+review%22)
and assign a Drake Developer.  This is intended to make sure that
requests from outside developers receive timely attention.  For a pull request
by a core Drake Developer, leaving it unassigned may be acceptable when it is
clearly an early work-in-progress -- but if it is unassigned for several days,
you should probably encourage the developer to label it "do not review" for
clarity.

Here's some sample text to post when a pull request does not have anyone
assigned yet:

* For PRs opened by a core Drake Developer:

  > Good day, ``@AUTHOR``.  This PR does not yet have a reviewer assigned.
  > Is it ready for review yet?
  > If yes, then please assign a feature reviewer.
  > If not, then please label it "status: do not review".

* For PRs opened by a new or infrequent contributor:

  > Good day, ``@AUTHOR``.  I'm assigning ``+@ASSIGNEE`` as the most relevant
  > team member to assist with this pull request.  If you already had a
  > different team member in mind, please let us know.  To ``@ASSIGNEE``,
  > feel free to delegate in case there is someone else more suitable.

For PRs assigned to you that have passed all commit checks (other than needing
a squash), merge the PR to master on behalf of the author -- unless it is
labeled "status: do not merge".
