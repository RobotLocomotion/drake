***********************
GitHub Issue Management
***********************

Drake uses GitHub issues to coordinate bug resolution and feature development.
There are typically a large number of open issues. To keep them organized,
issues are tagged with labels.  Each label uses the format ``group: value``,
where ``group`` is one of the following:

* ``team``: Indicates the engineering team that owns the issue.
* ``type``: Indicates the nature of the issue.
* ``priority``: Issues only. Indicates the urgency of resolution.
* ``configuration``: The supported configurations affected, if applicable.
* ``status``: PRs only.  Indicates the status of the PR.

Team
====
Every issue must have at least one ``team``. If no team agrees to own an issue
issue at the weekly tracker scrub, the issue will be closed. The teams and their
leads are:

* ``dynamics``: sherm1
* ``kitware``: billhoffman
* ``interfaces``: TBD
* ``optimization``: ggould-tri
* ``robot locomotion group``: RussTedrake
* ``sensors``: ErikSobel-TRI
* ``software core``: david-german-tri
* ``6.832``: RussTedrake

Type
====
Every issue must have at least one ``type``, and typically should have exactly
one. Issues that require a code change will typically have type ``bug``,
``feature request``, or ``cleanup``. There are a number of niche types for
other kinds of issues, and the exact set is expected to evolve over time.

Priority
========
The ``emergency`` priority indicates that the owning engineers should not work
on anything else until the issue is resolved. A postmortem document should be
opened at the same time as the ``emergency`` issue, and linked in the
description.

The ``high``, ``medium``, ``low``, and ``trivial`` priority levels have
semantics determined by the owning team. The following rules of thumb may be
useful:

* high-priority issues are planned to receive attention within the month.
* medium-priority issues are planned to receive attention within the quarter.
* low-priority issues may be planned for a subsequent quarter.
* trivial-priority issues will never be planned.

Configuration
=============
An issue may have configuration ``linux``, ``windows``, and/or ``mac``. It may
additionally have configuration ``matlab``.  If no ``configuration`` label is
present, the issue is assumed to affect all configurations.

Status
======
For the most part, we rely on reviewable.io to communicate PR status. There
are only three ``status`` labels: ``do not review``, ``do not merge``, and
``ready to merge``. All three flags are optional, but Drake administrators
managing the PR queue will respect them.
