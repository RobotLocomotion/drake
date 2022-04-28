---
title: GitHub Issue Management
---

Drake uses [GitHub issues](https://github.com/RobotLocomotion/drake/issues)
to coordinate bug resolution and feature development. We organize issues using
labels.  Each label uses the format ``group: value``, where ``group`` is one
of the following:

* ``component``: Indicates the primary affected feature area.
* ``type``: Indicates the nature of the issue.
* ``priority``: Indicates the urgency of resolution.
* ``configuration``: The supported configurations affected, if applicable.
* ``status``: PRs only.  Indicates the status of the PR.

Please only assign labels if you are reasonably confident they are correct.
The Drake development team will apply appropriate labels later as needed.

# Owner

Every issue must have at least one owner assigned.

# Component

Every issue must at least one ``component`` label assigned. Our preference is
to have exactly one ``component`` per issue, but we allow multiple in case
several components are equally relevant.

The components are:

* ``build system``
  * Bazel, CMake, dependencies, memory checkers, linters, etc.
  * lead: ``jwnimmer-tri``

* ``continuous integration``
  * Jenkins, CDash, mirroring of externals, Drake website, etc.
  * lead: ``BetsyMcPhail``

* ``distribution``
  * Nightly binaries, monthly releases, docker, installation
  via apt or brew, etc.
  * lead: ``jwnimmer-tri``

* ``geometry general``
  * Geometry infrastructure or topics that defy categorization into other geometry
  components.
  * lead: ``rpoyner-tri``

* ``geometry illustration``
  * What and how geometry gets communicated to external visualizers.
  * lead: ``joemasterjohn``

* ``geometry perception``
  * How geometry appears in color, depth, and label images (via the RenderEngine API).
  * lead: ``zachfang``

* ``geometry proximity``
  * Contact, distance, signed distance queries and related properties.
  * lead: ``DamrongGuoy``

* ``jupyter``
  * Topics relevant only when running inside a Python notebook. This label does
  not imply authoring tutorials (use ``component: tutorials`` for that).
  * lead: ``RussTedrake``

* ``mathematical program``
  * Formulating and solving mathematical programs through numerical optimization.
  * lead: ``hongkai-dai``
  * Typical directories:
    * ``drake/solvers``

* ``multibody plant``
  * MultibodyPlant and supporting code.
  * Typical directories:
    * ``drake/multibody/benchmarks``
    * ``drake/multibody/constraint``
    * ``drake/multibody/contact_solvers``
    * ``drake/multibody/hydroelastics``
    * ``drake/multibody/math``
    * ``drake/multibody/topology``
    * ``drake/multibody/tree``
    * ``drake/multibody/triangle_quadrature``
  * lead: ``amcastro-tri``

* ``multibody parsing``
  * Loading models into MultibodyPlant.
  * lead: ``sammy-tri``
  * Typical directories:
    * ``drake/multibody/parsing``

* ``planning and control``
  * Optimization-based planning and control, and search- and sampling-based
  planning.
  * lead: ``hongkai-dai``
  * Typical directories:
    * ``systems/controllers``
    * ``multibody/optimization``
    * ``geometry/optimization``

* ``softsim fem``
  * Deformable body simulation using Finite Element Method (FEM).
  * lead: ``xuchenhan-tri``
  * Typical directories:
    * ``drake/multibody/fem``

* ``pydrake``
  * Python API and its supporting Starlark macros.
  * lead: ``EricCousineau-TRI``
  * Typical directories:
    * ``drake/bindings/pydrake``
    * ``RobotLocomotion/pybind11``

* ``simulator``
  * Simulator, integrators, and supporting code.
  * lead: ``sherm1``
  * Typical directories:
    * ``drake/systems/analysis``

* ``system framework``
  * System, Context, and supporting code.
  * lead: ``sherm1``
  * Typical directories:
    * ``drake/systems/framework``

* ``tutorials``
  * Drake's tutorials and examples
  * lead: ``jwnimmer-tri``
  * Typical directories:
    * ``drake/tutorials``
    * ``drake/examples``

The responsiblities of the "lead" are to:

* Triage newly-filed issues to make sure that the issue:
  * is not a duplicate of an existing issue;
  * contains sufficient information to understand the problem; and
  * has a clear victory condition.
* Periodically revisit old issues to see what can be closed.

The lead is the primary point of contact for these tasks, but is free to
delegate the work to others.

# Priority

The ``emergency`` priority indicates that the involved parties should not work
on anything else until the issue is resolved.

The other priorities are determined by the lead of the assigned component. The
following rules of thumb may be useful for issues:

* ``priority: high`` - planned to receive attention within the month.
* ``priority: medium`` - planned to receive attention within the quarter.
* ``priority: low`` - planned for a subsequent quarter.
* ``priority: backlog`` - will be handled on an ad-hoc basis, as time permits.

# Configuration

An issue may have configuration label(s) such as ``linux``, ``mac``,
``python``, etc.  If no ``configuration`` label is present, the issue is
assumed to affect all configurations.

# Status

For the most part, we rely on reviewable.io to communicate PR status.
Status labels are optional, but Drake developers managing the PR queue
will respect them.

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

