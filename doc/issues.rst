***********************
GitHub Issue Management
***********************

Drake uses `GitHub issues <https://github.com/RobotLocomotion/drake/issues>`_
to coordinate bug resolution and feature development. We organize issues using
labels.  Each label uses the format ``group: value``, where ``group`` is one
of the following:

* ``team``: Indicates the engineering team that owns the issue.
* ``type``: Indicates the nature of the issue.
* ``priority``: Indicates the urgency of resolution.
* ``configuration``: The supported configurations affected, if applicable.
* ``status``: PRs only.  Indicates the status of the PR.

Please only assign labels if you are reasonably confident they are correct.
The Drake development team will apply appropriate labels to issues during
the weekly scrub.

Team
====
Every issue must have at least one ``team`` label. If no team agrees to own an
issue at the weekly tracker scrub, the issue will be closed with an explanation.
The teams, their leads, and their responsibilities are:

- ``dynamics``

  lead: sherm1

  responsibilities: physical accuracy, numerical methods, collision

- ``interfaces``

  lead: TBD

  responsibilities: integration with other robotics frameworks
- ``kitware``

  lead: billhoffman

  responsibilities: build, continuous integration
- ``optimization``

  lead: ggould-tri

  responsibilities: optimizers, solvers, symbolic analysis
- ``robot locomotion group``

  lead: RussTedrake

  responsibilities: MIT CSAIL research lab
- ``sensors``

  lead: ErikSobel-TRI

  responsibilities: modeling light, sound, and devices
- ``software core``

  lead: david-german-tri

  responsibilities: APIs, infrastructure, productivity
- ``6.832``

  lead: RussTedrake

  responsibilities: MIT underactuated robotics course

Type
====
Every issue must have at least one ``type``, and typically should have exactly
one. Issues that require a code change will typically have type ``bug``,
``feature request``, or ``cleanup``. There are a number of niche types for
other kinds of issues, and the exact set is expected to evolve over time.

Priority
========
The ``emergency`` priority indicates that the owning team should not work
on anything else until the issue is resolved. A postmortem document should be
opened at the same time as the ``emergency`` issue, linked in the description,
and updated as the situation evolves. Exception: broken builds are emergencies,
but a postmortem document is not required.

The ``high``, ``medium``, ``low``, and ``backlog`` priority levels have
semantics determined by the owning team. The following rules of thumb may be
useful:

* high-priority issues are planned to receive attention within the month.
* medium-priority issues are planned to receive attention within the quarter.
* low-priority issues may be planned for a subsequent quarter.
* backlog-priority issues will be handled on an ad-hoc basis, as time permits.

Configuration
=============
An issue may have configuration ``linux`` or ``mac``. It may additionally have
configuration ``matlab``.  If no ``configuration`` label is present, the issue
is assumed to affect all configurations.

Status
======
For the most part, we rely on reviewable.io to communicate PR status. There
are only two ``status`` labels.  Both flags are optional, but Drake
administrators managing the PR queue will respect them.

* ``do not review``: Use this status to indicate you do not want anyone to
  review your PR right now. This is useful if you created the PR to trigger
  CI and plan to iterate on the results. Even if this flag is absent, you
  are responsible for finding reviewers, as documented in `developers`.
  This flag simply protects you from unsolicited review.
* ``do not merge``: Use this status to indicate you do not want anyone to
  merge your PR right now, even if it passes all pre-merge checks. This is
  useful if you have minor post-LGTM changes to make, or if you need to
  coordinate the precise timing of the merge. If pre-merge checks are green
  and this flag is absent, a Drake administrator may merge your PR at any
  time.

