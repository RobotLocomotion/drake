---
title: GitHub Issue Management
---

Drake uses [GitHub issues](https://github.com/RobotLocomotion/drake/issues)
to coordinate bug resolution and feature development. We organize issues using
labels.  Each label uses the format ``group: value``, where ``group`` is one
of the following:

* ``team``: Indicates the engineering team that owns the issue.
* ``component``: Indicates the primary affected feature area.
* ``type``: Indicates the nature of the issue.
* ``priority``: Indicates the urgency of resolution.
* ``configuration``: The supported configurations affected, if applicable.
* ``status``: PRs only.  Indicates the status of the PR.

Please only assign labels if you are reasonably confident they are correct.
The Drake development team will apply appropriate labels later as needed.

# Owner

Every issue must have at least one owner assigned.

# Team

Every issue must have at least one ``team`` label. The teams, their leads, and
their responsibilities are:

* ``dynamics``
  * lead: sherm1
  * responsibilities: physical accuracy, numerical methods, collision,
  systems framework

* ``kitware``
  * lead: BetsyMcPhail
  * responsibilities: build, continuous integration

* ``manipulation``
  * lead: hongkai-dai
  * responsibilities: optimizers, solvers, symbolic analysis,
  ``drake/manipulation/`` subdirectory

* ``robot locomotion group``
  * lead: RussTedrake
  * responsibilities: examples/requests from MIT projects / MIT courses

# Component

Every issue must have at most one ``component`` label. The components are:

* ``build system``
  * Bazel, CMake, dependencies, memory checkers, linters, etc.
  * typical team: kitware

* ``continuous integration``
  * Jenkins, CDash, mirroring of externals, Drake website, etc.
  * typical team: kitware

* ``distribution``
  * Nightly binaries, monthly releases, docker, installation
  via apt or brew, etc.
  * typical team: kitware

* ``geometry externals``
  * VTK, FCL, drake_visualizer (core behavior), etc.
  * typical team: kitware

* ``geometry general``
  * Geometry infrastructure or topics that defy categorization into other geometry
  components.
  * typical team: dynamics

* ``geometry illustration``
  * What and how geometry gets communicated to external visualizers.
  * typical team: dynamics

* ``geometry perception``
  * How geometry appears in color, depth, and label images (via the RenderEngine API).
  * typical team: dynamics

* ``geometry proximity``
  * Contact, distance, signed distance queries and related properties.
  * typical team: dynamics

* ``jupyter``
  * Jupyter notebook infrastructure, Binder integration, etc.
  * *Note*: This label does not imply content authoring for tutorials.
  * typical team: kitware

* ``mathematical program``
  * Formulating and solving mathematical programs through numerical optimization,
  usually in ``drake/solvers``.
  * typical team: manipulation

* ``multibody plant``
  * MultibodyPlant and related code and documentation
  usually in ``drake/multibody``.
  * typical team: dynamics

* ``softsim fem``
  * Deformable body simulation using Finite Element Method (FEM) usually in
  ``drake/multibody/fem``.
  * typical team: dynamics

* ``pydrake``
  * Python API and documentation under ``//bindings/pydrake`` (and
  its supporting Starlark macros), the ``RobotLocomotion/pybind11`` fork, etc.
  * typical team: kitware

* ``simulator``
  * Simulator, integrators, and related code and documentation,
  usually in ``drake/systems/analysis``.
  * typical team: dynamics

* ``system framework``
  * System, Context, and related code and documentation,
  usually in ``drake/systems/framework``.
  * typical team: dynamics

# Priority

The ``emergency`` priority indicates that the owning team should not work
on anything else until the issue is resolved.

The other priorities are determined by the owning team. The following rules of
thumb may be useful for issues:

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

