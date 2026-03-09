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
<table>

<tr>
<th>Component</th>
<th>Description</th>
<th>Lead</th>
<th>Typical Directories</th>
</tr>

<tr>
<td><code>build system</code></td>
<td>Bazel, CMake, dependencies, memory checkers, linters.</td>
<td>jwnimmer-tri</td>
<td><small>
tools
</small></td>
</tr>

<tr>
<td><code>continuous&nbsp;integration</code></td>
<td>Jenkins, CDash, mirroring of externals, website infrastructure.</td>
<td>BetsyMcPhail</td>
<td><small>
RobotLocomotion/drake-ci
</small></td>
</tr>

<tr>
<td><code>distribution</code></td>
<td>Nightly binaries, monthly releases, installation.</td>
<td>jwnimmer-tri</td>
<td><small>
tools/install<br>
tools/wheel
</small></td>
</tr>

<tr>
<td><code>geometry general</code></td>
<td>Geometry infrastructure or topics that defy categorization into other
  geometry components.</td>
<td>SeanCurtis-TRI</td>
<td><small>
n/a
</small></td>
</tr>

<tr>
<td><code>geometry illustration</code></td>
<td>What and how geometry gets communicated to external visualizers.</td>
<td>joemasterjohn</td>
<td><small>
(portions of) geometry<br>
multibody/meshcat
</small></td>
</tr>

<tr>
<td><code>geometry perception</code></td>
<td>How geometry appears in color, depth, and label images (via the
  RenderEngine API).</td>
<td>SeanCurtis-TRI</td>
<td><small>
geometry/render<br>
systems/rendering
</small></td>
</tr>

<tr>
<td><code>geometry proximity</code></td>
<td>Contact, distance, signed distance queries and related properties.</td>
<td>SeanCurtis-TRI</td>
<td><small>
(portions of) geometry<br>
geometry/proximity
</small></td>
</tr>

<tr>
<td><code>graphs of convex sets</code></td>
<td>Graphs of Convex Sets and related algorithms</td>
<td>RussTedrake</td>
<td><small>
geometry/optimization<br>
planning/iris<br>
planning/trajectory_optimization/gcs_trajectory_optimization<br>
</small></td>
</tr>

<tr>
<td><code>jupyter</code></td>
<td>Topics relevant only when running inside a Python notebook. This label does
  not imply authoring tutorials (use component: tutorials for that).</td>
<td>RussTedrake</td>
<td><small>
n/a
</small></td>
</tr>

<tr>
<td><code>mathematical program</code></td>
<td>Formulating and solving mathematical programs; our autodiff and symbolic
  libraries.</td>
<td>hongkai-dai</td>
<td><small>
common/symbolic_**<br>
solvers
</small></td>
</tr>

<tr>
<td><code>messaging</code></td>
<td>Message-passing infrastructure, i.e., LCM.</td>
<td>sammy-tri</td>
<td><small>
lcm
systems/lcm
</small></td>
</tr>

<tr>
<td><code>multibody parsing</code></td>
<td>Loading models into MultibodyPlant.</td>
<td>rpoyner-tri</td>
<td><small>
multibody/parsing
</small></td>
</tr>

<tr>
<td><code>multibody plant</code></td>
<td>MultibodyPlant and supporting code.</td>
<td>joemasterjohn</td>
<td><small>
multibody/contact_solvers<br>
multibody/math<br>
multibody/plant<br>
multibody/tree<br>
</small></td>
</tr>

<tr>
<td><code>planning and control</code></td>
<td>Optimization-based planning and control, and search- and sampling-based
  planning.</td>
<td>hongkai-dai</td>
<td><small>
multibody/inverse_kinematics<br>
multibody/optimization<br>
planning<br>
systems/controllers<br>
systems/trajectory_optimization
</small></td>
</tr>

<tr>
<td><code>pydrake</code></td>
<td>Python API and its supporting Starlark macros.</td>
<td>rpoyner-tri</td>
<td><small>
bindings/pydrake<br>
RobotLocomotion/pybind11
</small></td>
</tr>

<tr>
<td><code>simulator</code></td>
<td>Simulator, integrators, and supporting code.</td>
<td>sherm1</td>
<td><small>
systems/analysis
</small></td>
</tr>

<tr>
<td><code>softsim fem</code></td>
<td>Deformable body simulation using Finite Element Method (FEM).</td>
<td>xuchenhan-tri</td>
<td><small>
multibody/fem
</small></td>
</tr>

<tr>
<td><code>system framework</code></td>
<td>System, Context, and supporting code.</td>
<td>sherm1</td>
<td><small>
systems/framework<br>
systems/primitives
</small></td>
</tr>

<tr>
<td><code>tutorials</code></td>
<td>Drake's tutorials, examples, and website content.</td>
<td>jwnimmer-tri</td>
<td><small>
examples<br>
tutorials
</small></td>
</tr>

</table>

The responsibilities of the "lead" are to:

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

