#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/planning/trajectory_optimization/direct_collocation.h"
// #include "drake/planning/trajectory_optimization/direct_transcription.h"
// #include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"
// #include "drake/planning/trajectory_optimization/integration_constraint.h"
// #include "drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h"
// #include "drake/planning/trajectory_optimization/multiple_shooting.h"
// #include "drake/planning/trajectory_optimization/sequential_expression_manager.h"

// Symbol: pydrake_doc_planning_trajectory_optimization
constexpr struct /* pydrake_doc_planning_trajectory_optimization */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::planning
    struct /* planning */ {
      // Symbol: drake::planning::trajectory_optimization
      struct /* trajectory_optimization */ {
        // Symbol: drake::planning::trajectory_optimization::AddDirectCollocationConstraint
        struct /* AddDirectCollocationConstraint */ {
          // Source: drake/planning/trajectory_optimization/direct_collocation.h
          const char* doc =
R"""(Helper method to add a DirectCollocationConstraint to the ``prog``,
ensuring that the order of variables in the binding matches the order
expected by the constraint.)""";
        } AddDirectCollocationConstraint;
        // Symbol: drake::planning::trajectory_optimization::DirectCollocation
        struct /* DirectCollocation */ {
          // Source: drake/planning/trajectory_optimization/direct_collocation.h
          const char* doc =
R"""(DirectCollocation implements the approach to trajectory optimization
as described in C. R. Hargraves and S. W. Paris. Direct trajectory
optimization using nonlinear programming and collocation. J Guidance,
10(4):338-342, July-August 1987. It assumes a first-order hold on the
input trajectory and a cubic spline representation of the state
trajectory, and adds dynamic constraints (and running costs) to the
midpoints as well as the breakpoints in order to achieve a 3rd order
integration accuracy.

Note: This algorithm only works with the continuous states of a
system.)""";
          // Symbol: drake::planning::trajectory_optimization::DirectCollocation::DirectCollocation
          struct /* ctor */ {
            // Source: drake/planning/trajectory_optimization/direct_collocation.h
            const char* doc =
R"""(Constructs the MathematicalProgram% and adds the collocation
constraints.

Parameter ``system``:
    A dynamical system to be used in the dynamic constraints. This
    system must support System∷ToAutoDiffXd. Note that this is aliased
    for the lifetime of this object.

Parameter ``context``:
    Required to describe any parameters of the system. The values of
    the state in this context do not have any effect. This context
    will also be "cloned" by the optimization; changes to the context
    after calling this method will NOT impact the trajectory
    optimization.

Parameter ``num_time_samples``:
    The number of breakpoints in the trajectory.

Parameter ``minimum_time_step``:
    Minimum spacing between sample times.

Parameter ``maximum_time_step``:
    Maximum spacing between sample times.

Parameter ``input_port_index``:
    A valid input port index for ``system`` or InputPortSelection. All
    other inputs on the system will be left disconnected (if they are
    disconnected in ``context)`` or will be fixed to their current
    values (if they are connected/fixed in ``context)``.

*Default:* kUseFirstInputIfItExists.
    $Parameter ``assume_non_continuous_states_are_fixed``:

Boolean which, if true, allows this algorithm to optimize without
considering the dynamics of any non-continuous states. This is helpful
for optimizing systems that might have some additional book-keeping
variables in their state. Only use this if you are sure that the
dynamics of the additional state variables cannot impact the dynamics
of the continuous states. $*Default:* false.

Parameter ``prog``:
    (optional). If non-null, then additional decision variables,
    costs, and constraints will be added into the existing
    MathematicalProgram. This can be useful for, e.g., combining
    multiple trajectory optimizations into a single program, coupled
    by a few constraints. If nullptr, then a new MathematicalProgram
    will be allocated.

Raises:
    RuntimeError if ``system`` is not supported by this direct
    collocation method.)""";
          } ctor;
          // Symbol: drake::planning::trajectory_optimization::DirectCollocation::ReconstructInputTrajectory
          struct /* ReconstructInputTrajectory */ {
            // Source: drake/planning/trajectory_optimization/direct_collocation.h
            const char* doc = R"""()""";
          } ReconstructInputTrajectory;
          // Symbol: drake::planning::trajectory_optimization::DirectCollocation::ReconstructStateTrajectory
          struct /* ReconstructStateTrajectory */ {
            // Source: drake/planning/trajectory_optimization/direct_collocation.h
            const char* doc = R"""()""";
          } ReconstructStateTrajectory;
        } DirectCollocation;
        // Symbol: drake::planning::trajectory_optimization::DirectCollocationConstraint
        struct /* DirectCollocationConstraint */ {
          // Source: drake/planning/trajectory_optimization/direct_collocation.h
          const char* doc =
R"""(Implements the direct collocation constraints for a first-order hold
on the input and a cubic polynomial representation of the state
trajectories.

Note that the DirectCollocation implementation allocates only ONE of
these constraints, but binds that constraint multiple times (with
different decision variables, along the trajectory).)""";
          // Symbol: drake::planning::trajectory_optimization::DirectCollocationConstraint::DirectCollocationConstraint
          struct /* ctor */ {
            // Source: drake/planning/trajectory_optimization/direct_collocation.h
            const char* doc_double =
R"""(See also:
    DirectCollocation constructor for a description of the parameters.

Raises:
    RuntimeError if ``system`` is not supported by this direct
    collocation method.)""";
            // Source: drake/planning/trajectory_optimization/direct_collocation.h
            const char* doc_autodiff =
R"""(Constructor which supports passing different mutable contexts for the
different evaluation times. This can be used to facilitate caching
(for instance, if the ``context_segment_start`` of one constraint uses
the ``context_segment_end`` of the previous constraint).

See also:
    DirectCollocation constructor for a description of the remaining
    parameters.

Raises:
    RuntimeError if ``system`` is not supported by this direct
    collocation method.)""";
          } ctor;
          // Symbol: drake::planning::trajectory_optimization::DirectCollocationConstraint::DoEval
          struct /* DoEval */ {
            // Source: drake/planning/trajectory_optimization/direct_collocation.h
            const char* doc = R"""()""";
          } DoEval;
          // Symbol: drake::planning::trajectory_optimization::DirectCollocationConstraint::num_inputs
          struct /* num_inputs */ {
            // Source: drake/planning/trajectory_optimization/direct_collocation.h
            const char* doc = R"""()""";
          } num_inputs;
          // Symbol: drake::planning::trajectory_optimization::DirectCollocationConstraint::num_states
          struct /* num_states */ {
            // Source: drake/planning/trajectory_optimization/direct_collocation.h
            const char* doc = R"""()""";
          } num_states;
        } DirectCollocationConstraint;
        // Symbol: drake::planning::trajectory_optimization::DirectTranscription
        struct /* DirectTranscription */ {
          // Source: drake/planning/trajectory_optimization/direct_transcription.h
          const char* doc =
R"""(DirectTranscription is perhaps the simplest implementation of a
multiple shooting method, where we have decision variables
representing the control and input at every sample time in the
trajectory, and one-step of numerical integration provides the dynamic
constraints between those decision variables.)""";
          // Symbol: drake::planning::trajectory_optimization::DirectTranscription::DirectTranscription
          struct /* ctor */ {
            // Source: drake/planning/trajectory_optimization/direct_transcription.h
            const char* doc_4args =
R"""(Constructs the MathematicalProgram and adds the dynamic constraints.
This version of the constructor is only for simple discrete-time
systems (with a single periodic time step update). Continuous-time
systems must call one of the constructors that takes bounds on the
time step as an argument.

Parameter ``system``:
    A dynamical system to be used in the dynamic constraints. This
    system must support System∷ToAutoDiffXd. Note that this is aliased
    for the lifetime of this object.

Parameter ``context``:
    Required to describe any parameters of the system. The values of
    the state in this context do not have any effect. This context
    will also be "cloned" by the optimization; changes to the context
    after calling this method will NOT impact the trajectory
    optimization.

Parameter ``num_time_samples``:
    The number of breakpoints in the trajectory.

Parameter ``input_port_index``:
    A valid input port index or valid InputPortSelection for
    ``system``. All other inputs on the system will be left
    disconnected (if they are disconnected in ``context)`` or will be
    set to their current values (if they are connected/fixed in
    ``context)``.

*Default:* kUseFirstInputIfItExists.
    $Raises:

RuntimeError if ``context.has_only_discrete_state() == false``.)""";
            // Source: drake/planning/trajectory_optimization/direct_transcription.h
            const char* doc_5args =
R"""(Constructs the MathematicalProgram and adds the dynamic constraints.
This version of the constructor is only for continuous-time systems;
the dynamics constraints use explicit forward Euler integration.

Parameter ``system``:
    A dynamical system to be used in the dynamic constraints. This
    system must support System∷ToAutoDiffXd. Note that this is aliased
    for the lifetime of this object.

Parameter ``context``:
    Required to describe any parameters of the system. The values of
    the state in this context do not have any effect. This context
    will also be "cloned" by the optimization; changes to the context
    after calling this method will NOT impact the trajectory
    optimization.

Parameter ``num_time_samples``:
    The number of breakpoints in the trajectory.

Parameter ``fixed_time_step``:
    The spacing between sample times.

Parameter ``input_port_index``:
    A valid input port index or valid InputPortSelection for
    ``system``. All other inputs on the system will be left
    disconnected (if they are disconnected in ``context)`` or will be
    set to their current values (if they are connected/fixed in
    ``context)``.

*Default:* kUseFirstInputIfItExists.
    $Raises:

RuntimeError if ``context.has_only_continuous_state() == false``.)""";
          } ctor;
          // Symbol: drake::planning::trajectory_optimization::DirectTranscription::ReconstructInputTrajectory
          struct /* ReconstructInputTrajectory */ {
            // Source: drake/planning/trajectory_optimization/direct_transcription.h
            const char* doc =
R"""(Get the input trajectory at the solution as a PiecewisePolynomial. The
order of the trajectory will be determined by the integrator used in
the dynamic constraints.)""";
          } ReconstructInputTrajectory;
          // Symbol: drake::planning::trajectory_optimization::DirectTranscription::ReconstructStateTrajectory
          struct /* ReconstructStateTrajectory */ {
            // Source: drake/planning/trajectory_optimization/direct_transcription.h
            const char* doc =
R"""(Get the state trajectory at the solution as a PiecewisePolynomial. The
order of the trajectory will be determined by the integrator used in
the dynamic constraints.)""";
          } ReconstructStateTrajectory;
        } DirectTranscription;
        // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization
        struct /* GcsTrajectoryOptimization */ {
          // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
          const char* doc =
R"""(GcsTrajectoryOptimization implements a simplified motion planning
optimization problem introduced in the paper `"Motion Planning around
Obstacles with Convex Optimization"
<https://arxiv.org/abs/2205.04422>`_ by Tobia Marcucci, Mark Petersen,
David von Wrangel, Russ Tedrake.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

Instead of using the full time-scaling curve, this problem uses a
single time-scaling variable for each region. This formulation yields
continuous trajectories, which are not differentiable at the
transition times between the regions since non-convex continuity
constraints are not supported yet. However, it supports continuity on
the path r(s) for arbitrary degree. The path r(s) can be reconstructed
from the gcs solution q(t) with ``NormalizeSegmentTimes()`` and
post-processed with e.g. Toppra to enforce acceleration bounds.

The ith piece of the composite trajectory is defined as q(t) = r((t -
tᵢ) / hᵢ). r : [0, 1] → ℝⁿ is a the path, parametrized as a Bézier
curve with order n. tᵢ and hᵢ are the initial time and duration of the
ith sub-trajectory.

This class supports the notion of a Subgraph of regions. This has
proven useful to facilitate multi-modal motion planning such as:
Subgraph A: find a collision-free trajectory for the robot to a
grasping posture, Subgraph B: find a collision-free trajectory for the
robot with the object in its hand to a placing posture, etc.

**** Continuous Revolute Joints

This class also supports robots with continuous revolute joints
(revolute joints that don't have any joint limits) and mobile bases.
Adding or subtracting 2π to such a joint's angle leaves it unchanged;
this logic is implemented behind the scenes. To use it, one should
specify the joint indices that don't have limits, and ensure all sets
satisfy the "convexity radius" property -- their width along a
dimension corresponding to a continuous revolute joint must be less
than π. This can be enforced when constructing the convex sets, or
after the fact with ``geometry∷optimization∷PartitionConvexSet``. The
``GcsTrajectoryOptimization`` methods ``AddRegions`` and ``AddEdges``
will handle all of the intersection checks behind the scenes,
including applying the appropriate logic to connect sets that "wrap
around" 2π.)""";
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::AddContinuityConstraints
          struct /* AddContinuityConstraints */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Enforces that for any two subsequent path segments in the entire
graph, the `continuity_order`th time derivative at the end of the
first segment equals that of the start of the second segment.

This adds a nonlinear constraint to the restriction and MIP
GraphOfConvexSets∷Transcription, while adding a convex surrogate to
the relaxation. For more details, see nonconvex_graph_of_convex_sets
"Guiding Non-convex Optimization with the GraphOfConvexSets".

The continuity is enforced on the control points of q(t), which appear
as nonlinear constraints.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (dᴺrᵤ(s=1) / dsᴺ) / hᵤᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥᴺ

.. raw:: html

    </details>

The convex surrogate is simply the path continuity, where hᵤᴺ and hᵥᴺ
are replaced by the characteristic times of the respective sets:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (dᴺrᵤ(s=1) / dsᴺ) / hᵤ₀ᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥ₀ᴺ

.. raw:: html

    </details>

. For now, these are set to one, but future work may involve scaling
them by the size of the sets.

Parameter ``continuity_order``:
    is the order of the continuity constraint.

Raises:
    RuntimeError if the continuity order is less than one since path
    continuity is enforced by default.

Note:
    To enforce that the trajectory is of class C^k, you must call
    AddContinuityConstraint for each continuity_order 1 through k.)""";
          } AddContinuityConstraints;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::AddEdges
          struct /* AddEdges */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Connects two subgraphs with directed edges.

Parameter ``from_subgraph``:
    is the subgraph to connect from. Must have been created from a
    call to AddRegions() on this object, not some other optimization
    program.

Parameter ``to_subgraph``:
    is the subgraph to connect to. Must have been created from a call
    to AddRegions() on this object, not some other optimization
    program.

Parameter ``subspace``:
    is the subspace that the connecting control points must be in.
    Subspace is optional. Only edges that connect through the subspace
    will be added, and the subspace is added as a constraint on the
    connecting control points. Subspaces of type point or HPolyhedron
    are supported since other sets require constraints that are not
    yet supported by the GraphOfConvexSets∷Edge constraint, e.g., set
    containment of a Hyperellipsoid is formulated via LorentzCone
    constraints. Workaround: Create a subgraph of zero order with the
    subspace as the region and connect it between the two subgraphs.
    This works because GraphOfConvexSet∷Vertex supports arbitrary
    instances of ConvexSets.

Parameter ``edges_between_regions``:
    can be used to manually specify which edges should be added,
    avoiding the intersection checks. It should be a list of tuples
    ``(i,j)``, where an edge will be added from the ``i`th index
    region in `from_subgraph`` to the ``j`th index region in
    `to_subgraph``.

Parameter ``edge_offsets``:
    is an optional list of vectors. If defined, the list must contain
    the same number of entries as ``edges_between_regions``, and the
    order must match. In other words, if defined, there must be one
    edge offset for each specified edge, and they must be at the same
    index. For each pair of sets listed in ``edges_between_regions``,
    the first set is translated (in configuration space) by the
    corresponding vector in edge_offsets before computing the
    constraints associated to that edge. This is used to add edges
    between sets that "wrap around" 2π along some dimension, due to,
    e.g., a continuous revolute joint. This edge offset corresponds to
    the translation component of the affine map τ_uv in equation (11)
    of "Non-Euclidean Motion Planning with Graphs of
    Geodesically-Convex Sets", and per the discussion in Subsection VI
    A, τ_uv has no rotation component. If edge_offsets is nullptr, it
    will instead be computed automatically.

Raises:
    RuntimeError if ``edge_offsets`` is provided, but
    ``edge_offsets.size() != edges_between_regions.size()``.)""";
          } AddEdges;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::AddNonlinearDerivativeBounds
          struct /* AddNonlinearDerivativeBounds */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Adds a nonlinear derivative constraints to the entire graph ``lb`` ≤
dᴺq(t) / dtᴺ ≤ ``ub``.

This adds a nonlinear constraint to the restriction and MIP
GraphOfConvexSets∷Transcription, while adding a convex surrogate to
the relaxation. For more details, see nonconvex_graph_of_convex_sets
"Guiding Non-convex Optimization with the GraphOfConvexSets".

The nonlinear constraint involves the derivative dᴺq(t) / dtᴺ which is
decomposed as dᴺr(s) / dsᴺ / hᴺ. The convex surrogate replaces the
nonlinear component hᴺ with h₀ᴺ⁻¹h, where h₀ is the characteristic
time of the set. For now, h₀ is set to 1.0 for all sets.

Parameter ``lb``:
    is the lower bound of the derivative.

Parameter ``ub``:
    is the upper bound of the derivative.

Parameter ``derivative_order``:
    is the order of the derivative to be constrained.

Raises:
    RuntimeError if lb or ub are not of size num_positions().

Raises:
    RuntimeError if the derivative order <= 1, since the linear
    velocity bounds are preferred.)""";
          } AddNonlinearDerivativeBounds;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::AddPathContinuityConstraints
          struct /* AddPathContinuityConstraints */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Enforces that for any two subsequent path segments in the entire
graph, the ``continuity_order`th path derivative at the end of the
first segment equals that of the start of the second segment.

Note that the constraints are on the control points of the derivatives
of r(s) and not q(t). This may result in discontinuities of the
trajectory return by `SolvePath()`` since the r(s) will get rescaled
by the duration h to yield q(t). ``NormalizeSegmentTimes()`` will
return r(s) with valid continuity.

Parameter ``continuity_order``:
    is the order of the continuity constraint.

Raises:
    RuntimeError if the continuity order is less than one since path
    continuity is enforced by default.

Note:
    To enforce that the trajectory is of class C^k, you must call
    AddPathContinuityConstraint for each continuity_order 1 through k.)""";
          } AddPathContinuityConstraints;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::AddPathEnergyCost
          struct /* AddPathEnergyCost */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc_1args_weight_matrix =
R"""(Similar to AddPathLengthCost in usage, but minimizes ∑ |weight_matrix
* (rᵢ₊₁ − rᵢ)|₂². In comparison to AddPathLength cost, this cost
encourages control points to be evenly spaced but may result in
greater number of regions and larger path length on the solution. It
is recommended to use this cost only with SolveConvexRestriction when
it becomes a quadratic cost for which some solvers show a better
performance.

This cost will be added to the entire graph. Since the path length is
only defined for Bézier curves that have two or more control points,
this cost will only added to all subgraphs with order greater than
zero. Note that this cost will be applied even to subgraphs added in
the future.

Parameter ``weight_matrix``:
    is the relative weight of each component for the cost. The
    diagonal of the matrix is the weight for each dimension. The
    off-diagonal elements are the weight for the cross terms, which
    can be used to penalize diagonal movement.

Precondition:
    weight_matrix must be of size num_positions() x num_positions().)""";
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc_1args_weight =
R"""(Similar to AddPathLengthCost in usage, but minimizes ∑ |(rᵢ₊₁ − rᵢ)|₂²
with weight being applied uniformly to all dimensions. In comparison
to AddPathLength cost, this cost encourages control points to be
evenly spaced but may result in greater number of regions and larger
path length on the solution. It is recommended to use this cost only
with SolveConvexRestriction when it becomes a quadratic cost for which
some solvers show a better performance.

This cost will be added to the entire graph. Since the path length is
only defined for Bézier curves that have two or more control points,
this cost will only added to all subgraphs with order greater than
zero. Note that this cost will be applied even to subgraphs added in
the future.

Parameter ``weight``:
    is the relative weight of the cost.)""";
          } AddPathEnergyCost;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::AddPathLengthCost
          struct /* AddPathLengthCost */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc_1args_weight_matrix =
R"""(Adds multiple L2Norm Costs on the upper bound of the path length.
Since we cannot directly compute the path length of a Bézier curve, we
minimize the upper bound of the path integral by minimizing the sum of
(weighted) distances between control points: ∑ |weight_matrix * (rᵢ₊₁
− rᵢ)|₂.

This cost will be added to the entire graph. Since the path length is
only defined for Bézier curves that have two or more control points,
this cost will only added to all subgraphs with order greater than
zero. Note that this cost will be applied even to subgraphs added in
the future.

Parameter ``weight_matrix``:
    is the relative weight of each component for the cost. The
    diagonal of the matrix is the weight for each dimension. The
    off-diagonal elements are the weight for the cross terms, which
    can be used to penalize diagonal movement.

Precondition:
    weight_matrix must be of size num_positions() x num_positions().)""";
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc_1args_weight =
R"""(Adds multiple L2Norm Costs on the upper bound of the path length.
Since we cannot directly compute the path length of a Bézier curve, we
minimize the upper bound of the path integral by minimizing the sum of
distances between control points. For Bézier curves, this is
equivalent to the sum of the L2Norm of the derivative control points
of the curve divided by the order.

This cost will be added to the entire graph. Since the path length is
only defined for Bézier curves that have two or more control points,
this cost will only added to all subgraphs with order greater than
zero. Note that this cost will be applied even to subgraphs added in
the future.

Parameter ``weight``:
    is the relative weight of the cost.)""";
          } AddPathLengthCost;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::AddRegions
          struct /* AddRegions */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc_7args =
R"""(Creates a Subgraph with the given regions and indices.

Parameter ``regions``:
    represent the valid set a control point can be in. We retain a
    copy of the regions since other functions may access them. If any
    of the positions represent revolute joints without limits, each
    region has a maximum width of strictly less than π along
    dimensions corresponding to those joints.

Parameter ``edges_between_regions``:
    is a list of pairs of indices into the regions vector. For each
    pair representing an edge between two regions, an edge is added
    within the subgraph. Note that the edges are directed so (i,j)
    will only add an edge from region i to region j.

Parameter ``order``:
    is the order of the Bézier curve.

Parameter ``h_max``:
    is the maximum duration to spend in a region (seconds). Some
    solvers struggle numerically with large values.

Parameter ``h_min``:
    is the minimum duration to spend in a region (seconds) if that
    region is visited on the optimal path. Some cost and constraints
    are only convex for h > 0. For example the perspective quadratic
    cost of the path energy ||ṙ(s)||² / h becomes non-convex for h =
    0. Otherwise h_min can be set to 0.

Parameter ``name``:
    is the name of the subgraph. If the passed name is an empty
    string, a default name will be provided.

Parameter ``edge_offsets``:
    is an optional list of vectors. If defined, the list must contain
    the same number of entries as ``edges_between_regions``. For each
    pair of sets listed in ``edges_between_regions``, the first set is
    translated (in configuration space) by the corresponding vector in
    edge_offsets before computing the constraints associated to that
    edge. This is used to add edges between sets that "wrap around" 2π
    along some dimension, due to, e.g., a continuous revolute joint.
    This edge offset corresponds to the translation component of the
    affine map τ_uv in equation (11) of "Non-Euclidean Motion Planning
    with Graphs of Geodesically-Convex Sets", and per the discussion
    in Subsection VI A, τ_uv has no rotation component. If
    edge_offsets is nullptr, it will instead be computed
    automatically.

Raises:
    RuntimeError if any index referenced in ``edges_between_regions``
    is outside the range [0, ssize(regions)).)""";
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc_5args =
R"""(Creates a Subgraph with the given regions. This function will compute
the edges between the regions based on the set intersections.

Parameter ``regions``:
    represent the valid set a control point can be in. We retain a
    copy of the regions since other functions may access them. If any
    of the positions represent continuous revolute joints, each region
    must have a maximum width of strictly less than π along dimensions
    corresponding to those joints.

Parameter ``order``:
    is the order of the Bézier curve.

Parameter ``h_min``:
    is the minimum duration to spend in a region (seconds) if that
    region is visited on the optimal path. Some cost and constraints
    are only convex for h > 0. For example the perspective quadratic
    cost of the path energy ||ṙ(s)||² / h becomes non-convex for h =
    0. Otherwise h_min can be set to 0.

Parameter ``h_max``:
    is the maximum duration to spend in a region (seconds). Some
    solvers struggle numerically with large values.

Parameter ``name``:
    is the name of the subgraph. A default name will be provided.

Raises:
    RuntimeError if any of the regions has a width of π or greater
    along dimensions corresponding to continuous revolute joints.)""";
          } AddRegions;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::AddTimeCost
          struct /* AddTimeCost */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Adds a minimum time cost to all regions in the whole graph. The cost
is the sum of the time scaling variables.

This cost will be added to the entire graph. Note that this cost will
be applied even to subgraphs added in the future.

Parameter ``weight``:
    is the relative weight of the cost.)""";
          } AddTimeCost;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::AddVelocityBounds
          struct /* AddVelocityBounds */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Adds a linear velocity constraint to the entire graph ``lb`` ≤ q̇(t) ≤
``ub``.

Parameter ``lb``:
    is the lower bound of the velocity.

Parameter ``ub``:
    is the upper bound of the velocity.

This constraint will be added to the entire graph. Since the velocity
requires forming the derivative of the Bézier curve, this constraint
will only added to all subgraphs with order greater than zero. Note
that this constraint will be applied even to subgraphs added in the
future.

Raises:
    RuntimeError if lb or ub are not of size num_positions().)""";
          } AddVelocityBounds;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs
          struct /* EdgesBetweenSubgraphs */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(EdgesBetweenSubgraphs are defined as the connecting edges between two
given subgraphs. These edges are a subset of the many other edges in
the larger graph. From an API standpoint, EdgesBetweenSubgraphs enable
transitions between Subgraphs, which can enable transitions between
modes. Further, it allows different constraints to be added in the
transition between subgraphs. Note that the EdgesBetweenSubgraphs
can't be separated from the actual edges in the GraphOfConvexSets
framework, thus mixing it with other instances of
GCSTrajetoryOptimization is not supported.)""";
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::AddContinuityConstraints
            struct /* AddContinuityConstraints */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Enforces that for any two subsequent path segments that are joined by
an edge in this EdgesBetweenSubgraphs, the `continuity_order`th time
derivative at the end of the first segment equals that of the start of
the second segment.

This adds a nonlinear constraint to the restriction and MIP
GraphOfConvexSets∷Transcription, while adding a convex surrogate to
the relaxation. For more details, see nonconvex_graph_of_convex_sets
"Guiding Non-convex Optimization with the GraphOfConvexSets".

The continuity is enforced on the control points of q(t), which appear
as nonlinear constraints.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (dᴺrᵤ(s=1) / dsᴺ) / hᵤᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥᴺ

.. raw:: html

    </details>

The convex surrogate is simply the path continuity, where hᵤᴺ and hᵥᴺ
are replaced by the characteristic times of the respective sets:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (dᴺrᵤ(s=1) / dsᴺ) / hᵤ₀ᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥ₀ᴺ

.. raw:: html

    </details>

. For now, these are set to one, but future work may involve scaling
them by the size of the sets.

Parameter ``continuity_order``:
    is the order of the continuity constraint.

Raises:
    RuntimeError if the continuity order is not equal or less than the
    order of both subgraphs.

Raises:
    RuntimeError if the continuity order is less than one since path
    continuity is enforced by default.

Note:
    To enforce that the trajectory is of class C^k, you must call
    AddContinuityConstraint for each continuity_order 1 through k.)""";
            } AddContinuityConstraints;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::AddEdgeConstraint
            struct /* AddEdgeConstraint */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_e_use_in_transcription =
R"""(Adds an arbitrary user-defined constraint to every edge within the
EdgesBetweenSubgraphs. The constraint should be defined using the
placeholder control point variables (obtained from
edge_constituent_vertex_control_points()) and the placeholder time
scaling variables (obtained from edge_constituent_vertex_durations()).
This enables greater modeling freedom, but we cannot guarantee a
feasible solution for all possible constraints.

Raises:
    RuntimeError if any variables besides those from
    edge_constituent_vertex_durations and
    edge_constituent_vertex_control_points are used.

Constraints which do not support the perspective operation cannot be
used with Transcription∷kMIP or Transcription∷kRelaxation. Consider
providing an appropriate "convex surrogate" that is supported within
GraphOfConvexSets, or exclusively using the SolveConvexRestriction
method.)""";
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_binding_use_in_transcription =
R"""(Convenience overload of AddEdgeConstraint to take in a
Binding<Constraint>.

Raises:
    RuntimeError if any variables besides those from
    edge_constituent_vertex_durations and
    edge_constituent_vertex_control_points are used.)""";
            } AddEdgeConstraint;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::AddEdgeCost
            struct /* AddEdgeCost */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_e_use_in_transcription =
R"""(Adds an arbitrary user-defined cost to every edge within the
EdgesBetweenSubgraphs. The cost should be defined using the
placeholder control point variables (obtained from
edge_constituent_vertex_control_points()) and the placeholder time
scaling variables (obtained from edge_constituent_vertex_durations()).
This enables greater modeling freedom, but we cannot guarantee a
feasible solution for all possible costs.

Raises:
    RuntimeError if any variables besides those from
    edge_constituent_vertex_durations and
    edge_constituent_vertex_control_points are used.

Costs which do not support the perspective operation cannot be used
with Transcription∷kMIP or Transcription∷kRelaxation. Consider
providing an appropriate "convex surrogate" that is supported within
GraphOfConvexSets, or exclusively using the SolveConvexRestriction
method.)""";
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_binding_use_in_transcription =
R"""(Convenience overload of AddEdgeCost to take in a Binding<Cost>.

Raises:
    RuntimeError if any variables besides those from
    edge_constituent_vertex_durations and
    edge_constituent_vertex_control_points are used.)""";
            } AddEdgeCost;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::AddNonlinearDerivativeBounds
            struct /* AddNonlinearDerivativeBounds */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Adds a nonlinear derivative constraints to the control point
connecting the subgraphs ``lb`` ≤ dᴺq(t) / dtᴺ ≤ ``ub``.

This adds a nonlinear constraint to the restriction and MIP
GraphOfConvexSets∷Transcription, while adding a convex surrogate to
the relaxation. For more details, see nonconvex_graph_of_convex_sets
"Guiding Non-convex Optimization with the GraphOfConvexSets".

The nonlinear constraint involves the derivative dᴺq(t) / dtᴺ which is
decomposed as dᴺr(s) / dsᴺ / hᴺ. The convex surrogate replaces the
nonlinear component hᴺ with h₀ᴺ⁻¹h, where h₀ is the characteristic
time of the set. For now, h₀ is set to 1.0 for all sets.

Parameter ``lb``:
    is the lower bound of the derivative.

Parameter ``ub``:
    is the upper bound of the derivative.

Parameter ``derivative_order``:
    is the order of the derivative to be constrained.

Raises:
    RuntimeError if both subgraphs order is less than the desired
    derivative order.

Raises:
    RuntimeError if the derivative order <= 1, since the linear
    velocity bounds are preferred.

Raises:
    RuntimeError if lb or ub are not of size num_positions().)""";
            } AddNonlinearDerivativeBounds;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::AddPathContinuityConstraints
            struct /* AddPathContinuityConstraints */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Enforces that for any two subsequent path segments that are joined by
an edge in this EdgesBetweenSubgraphs, the ``continuity_order`th path
derivative at the end of the first segment equals that of the start of
the second segment.

Note that the constraints are on the control points of the derivatives
of r(s) and not q(t). This may result in discontinuities of the
trajectory return by `SolvePath()`` since the r(s) will get rescaled
by the duration h to yield q(t). ``NormalizeSegmentTimes()`` will
return r(s) with valid continuity.

Parameter ``continuity_order``:
    is the order of the continuity constraint.

Raises:
    RuntimeError if the continuity order is not equal or less than the
    order of both subgraphs.

Raises:
    RuntimeError if the continuity order is less than one since path
    continuity is enforced by default.

Note:
    To enforce that the trajectory is of class C^k, you must call
    AddPathContinuityConstraint for each continuity_order 1 through k.)""";
            } AddPathContinuityConstraints;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::AddVelocityBounds
            struct /* AddVelocityBounds */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Adds a linear velocity constraint to the control point connecting the
subgraphs ``lb`` ≤ q̇(t) ≤ ``ub``.

Parameter ``lb``:
    is the lower bound of the velocity.

Parameter ``ub``:
    is the upper bound of the velocity.

Raises:
    RuntimeError if both subgraphs order is zero, since the velocity
    is defined as the derivative of the Bézier curve. At least one of
    the subgraphs must have an order of at least 1.

Raises:
    RuntimeError if lb or ub are not of size num_positions().)""";
            } AddVelocityBounds;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::AddZeroDerivativeConstraints
            struct /* AddZeroDerivativeConstraints */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Enforces zero derivatives on the control point connecting the
subgraphs.

For velocity, acceleration, jerk, etc. enforcing zero-derivative on
the trajectory q(t) is equivalent to enforcing zero-derivative on the
trajectory r(s). Hence this constraint is convex.

Parameter ``derivative_order``:
    is the order of the derivative to be constrained.

Raises:
    RuntimeError if the derivative order < 1.

Raises:
    RuntimeError if both subgraphs order is less than the desired
    derivative order.)""";
            } AddZeroDerivativeConstraints;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::Edges
            struct /* Edges */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns constant reference to a vector of mutable pointers to the
edges.)""";
            } Edges;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::EdgesBetweenSubgraphs
            struct /* ctor */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::edge_constituent_vertex_control_points
            struct /* edge_constituent_vertex_control_points */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns a pair of placeholder decision variables (not actually
declared as decision variables in the MathematicalProgram) associated
with the control points of the trajectory in two sets that are
connected by an edge from this EdgesBetweenSubgraphs. Each variable
will be of shape (num_positions(), order+1), where the ith column is
the ith control point. (Note that the first and second variable will
have different shapes if the order of the two subgraphs is different.)
These variables will be substituted for real decision variables in
methods like AddEdgeCost and AddEdgeConstraint. Passing this variable
directly into objectives/constraints will result in an error.)""";
            } edge_constituent_vertex_control_points;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EdgesBetweenSubgraphs::edge_constituent_vertex_durations
            struct /* edge_constituent_vertex_durations */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns a pair of placeholder decision variables (not actually
declared as decision variables in the MathematicalProgram) associated
with the time scaling of the trajectory in two sets that are connected
by an edge from this EdgesBetweenSubgraphs. These variables will be
substituted for real decision variables in methods like AddEdgeCost
and AddEdgeConstraint. Passing this variable directly into
objectives/constraints will result in an error.)""";
            } edge_constituent_vertex_durations;
          } EdgesBetweenSubgraphs;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::EstimateComplexity
          struct /* EstimateComplexity */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Provide a heuristic estimate of the complexity of the underlying GCS
mathematical program, for regression testing purposes. Here we sum the
total number of variable appearances in our costs and constraints as a
rough approximation of the complexity of the subproblems.)""";
          } EstimateComplexity;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::GcsTrajectoryOptimization
          struct /* ctor */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Constructs the motion planning problem.

Parameter ``num_positions``:
    is the dimension of the configuration space.

Parameter ``continuous_revolute_joints``:
    is a list of indices corresponding to continuous revolute joints,
    i.e., revolute joints which don't have any joint limits, and hence
    "wrap around" at 2π. Each entry in continuous_revolute_joints must
    be non-negative, less than num_positions, and unique. This feature
    is currently only supported within a single subgraph: continuous
    revolute joints won't be taken into account when constructing
    edges between subgraphs or checking if sets intersect through a
    subspace.)""";
          } ctor;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::GetEdgesBetweenSubgraphs
          struct /* GetEdgesBetweenSubgraphs */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Returns a vector of all edges between subgraphs.)""";
          } GetEdgesBetweenSubgraphs;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::GetGraphvizString
          struct /* GetGraphvizString */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Returns a Graphviz string describing the graph vertices and edges. If
``result`` is supplied, then the graph will be annotated with the
solution values.)""";
          } GetGraphvizString;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::GetSubgraphs
          struct /* GetSubgraphs */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc = R"""(Returns a vector of all subgraphs.)""";
          } GetSubgraphs;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::NormalizeSegmentTimes
          struct /* NormalizeSegmentTimes */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Normalizes each trajectory segment to one second in duration.
Reconstructs the path r(s) from the solution trajectory q(t) of
``SolvePath()`` s.t. each segment of the resulting trajectory will be
one second long. The start time will match the original start time.

Parameter ``trajectory``:
    The solution trajectory returned by ``SolvePath()``.

Raises:
    RuntimeError if not all trajectory segments of the
    CompositeTrajectory are of type BezierCurve<double>)""";
          } NormalizeSegmentTimes;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::RemoveSubgraph
          struct /* RemoveSubgraph */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Remove a subgraph and all associated edges found in the subgraph and
to and from other subgraphs.

Precondition:
    The subgraph must have been created from a call to AddRegions() on
    this object.)""";
          } RemoveSubgraph;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::SolveConvexRestriction
          struct /* SolveConvexRestriction */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Solves a trajectory optimization problem through specific vertices.

This method allows for targeted optimization by considering only
selected active vertices, reducing the problem's complexity. See
geometry∷optimization∷GraphOfConvexSets∷SolveConvexRestriction(). This
API prefers a sequence of vertices over edges, as a user may know
which regions the solution should pass through.
GcsTrajectoryOptimization∷AddRegions() automatically manages edge
creation and intersection checks, which makes passing a sequence of
edges less convenient.

Parameter ``active_vertices``:
    A sequence of ordered vertices of subgraphs to be included in the
    problem.

Parameter ``options``:
    include all settings for solving the shortest path problem.

Precondition:
    There must be at least two vertices in active_vertices.

Raises:
    RuntimeError if the vertices are not connected.

Raises:
    RuntimeError if two vertices are connected by multiple edges. This
    may happen if one connects two graphs through multiple subspaces,
    which is currently not supported with this method.

Raises:
    RuntimeError if the program cannot be written as a convex
    optimization consumable by one of the standard solvers.)""";
          } SolveConvexRestriction;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::SolvePath
          struct /* SolvePath */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Formulates and solves the mixed-integer convex formulation of the
shortest path problem on the whole graph.

See also:
    ``geometry∷optimization∷GraphOfConvexSets∷SolveShortestPath()``
    for further details.

Parameter ``source``:
    specifies the source subgraph. Must have been created from a call
    to AddRegions() on this object, not some other optimization
    program. If the source is a subgraph with more than one region, an
    empty set will be added and optimizer will choose the best region
    to start in. To start in a particular point, consider adding a
    subgraph of order zero with a single region of type Point.

Parameter ``target``:
    specifies the target subgraph. Must have been created from a call
    to AddRegions() on this object, not some other optimization
    program. If the target is a subgraph with more than one region, an
    empty set will be added and optimizer will choose the best region
    to end in. To end in a particular point, consider adding a
    subgraph of order zero with a single region of type Point.

Parameter ``options``:
    include all settings for solving the shortest path problem. The
    following default options will be used if they are not provided in
    ``options``: - `options.convex_relaxation = true`, -
    ``options.max_rounded_paths = 5``, - `options.preprocessing =
    true`.

See also:
    ``geometry∷optimization∷GraphOfConvexSetsOptions`` for further
    details.)""";
          } SolvePath;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph
          struct /* Subgraph */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(A Subgraph is a subset of the larger graph. It is defined by a set of
regions and edges between them based on intersection. From an API
standpoint, a Subgraph is useful to define a multi-modal motion
planning problem. Further, it allows different constraints and objects
to be added to different subgraphs. Note that the the
GraphOfConvexSets does not differentiate between subgraphs and can't
be mixed with other instances of GcsTrajectoryOptimization.)""";
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddContinuityConstraints
            struct /* AddContinuityConstraints */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Enforces that for any two subsequent path segments within the
subgraph, the `continuity_order`th time derivative at the end of the
first segment equals that of the start of the second segment.

This adds a nonlinear constraint to the restriction and MIP
GraphOfConvexSets∷Transcription, while adding a convex surrogate to
the relaxation. For more details, see nonconvex_graph_of_convex_sets
"Guiding Non-convex Optimization with the GraphOfConvexSets".

The continuity is enforced on the control points of q(t), which appear
as nonlinear constraints.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (dᴺrᵤ(s=1) / dsᴺ) / hᵤᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥᴺ

.. raw:: html

    </details>

The convex surrogate is simply the path continuity, where hᵤᴺ and hᵥᴺ
are replaced by the characteristic times of the respective sets:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (dᴺrᵤ(s=1) / dsᴺ) / hᵤ₀ᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥ₀ᴺ

.. raw:: html

    </details>

. For now, these are set to one, but future work may involve scaling
them by the size of the sets.

Parameter ``continuity_order``:
    is the order of the continuity constraint.

Raises:
    RuntimeError if the continuity order is not equal or less than the
    order the subgraphs.

Raises:
    RuntimeError if the continuity order is less than one since path
    continuity is enforced by default.

Note:
    To enforce that the trajectory is of class C^k, you must call
    AddContinuityConstraint for each continuity_order 1 through k.)""";
            } AddContinuityConstraints;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddEdgeConstraint
            struct /* AddEdgeConstraint */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_e_use_in_transcription =
R"""(Adds an arbitrary user-defined constraint (in the form of a Formula or
Binding<Constraint>) to every internal edge within the subgraph. The
constraint should be defined using the placeholder control point
variables (obtained from edge_constituent_vertex_control_points()) and
the placeholder time scaling variables (obtained from
edge_constituent_vertex_durations()). This enables greater modeling
freedom, but we cannot guarantee a feasible solution for all possible
constraints.

Raises:
    RuntimeError if any variables besides those from
    edge_constituent_vertex_durations and
    edge_constituent_vertex_control_points are used.

Constraints which do not support the perspective operation cannot be
used with Transcription∷kMIP or Transcription∷kRelaxation. Consider
providing an appropriate "convex surrogate" that is supported within
GraphOfConvexSets, or exclusively using the SolveConvexRestriction
method.)""";
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_binding_use_in_transcription =
R"""(Convenience overload of AddEdgeConstraint to take in a
Binding<Constraint>.

Raises:
    RuntimeError if any variables besides those from
    edge_constituent_vertex_durations and
    edge_constituent_vertex_control_points are used.)""";
            } AddEdgeConstraint;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddEdgeCost
            struct /* AddEdgeCost */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_e_use_in_transcription =
R"""(Adds an arbitrary user-defined cost to every internal edge within the
subgraph. The cost should be defined using the placeholder control
point variables (obtained from
edge_constituent_vertex_control_points()) and the placeholder time
scaling variables (obtained from edge_constituent_vertex_durations()).
This enables greater modeling freedom, but we cannot guarantee a
feasible solution for all possible costs.

Raises:
    RuntimeError if any variables besides those from
    edge_constituent_vertex_durations and
    edge_constituent_vertex_control_points are used.

Costs which do not support the perspective operation cannot be used
with Transcription∷kMIP or Transcription∷kRelaxation. Consider
providing an appropriate "convex surrogate" that is supported within
GraphOfConvexSets, or exclusively using the SolveConvexRestriction
method.)""";
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_binding_use_in_transcription =
R"""(Convenience overload of AddEdgeCost to take in a Binding<Cost>.

Raises:
    RuntimeError if any variables besides those from
    edge_constituent_vertex_durations and
    edge_constituent_vertex_control_points are used.)""";
            } AddEdgeCost;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddNonlinearDerivativeBounds
            struct /* AddNonlinearDerivativeBounds */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Adds a nonlinear derivative constraints to the subgraph ``lb`` ≤
dᴺq(t) / dtᴺ ≤ ``ub``.

This adds a nonlinear constraint to the restriction and MIP
GraphOfConvexSets∷Transcription, while adding a convex surrogate to
the relaxation. For more details, see nonconvex_graph_of_convex_sets
"Guiding Non-convex Optimization with the GraphOfConvexSets".

The nonlinear constraint involves the derivative dᴺq(t) / dtᴺ which is
decomposed as dᴺr(s) / dsᴺ / hᴺ. The convex surrogate replaces the
nonlinear component hᴺ with h₀ᴺ⁻¹h, where h₀ is the characteristic
time of the set. For now, h₀ is set to 1.0 for all sets.

Parameter ``lb``:
    is the lower bound of the derivative.

Parameter ``ub``:
    is the upper bound of the derivative.

Parameter ``derivative_order``:
    is the order of the derivative to be constrained.

Raises:
    RuntimeError if subgraph order is less than the derivative order.

Raises:
    RuntimeError if the derivative order <= 1, since the linear
    velocity bounds are preferred.

Raises:
    RuntimeError if lb or ub are not of size num_positions().)""";
            } AddNonlinearDerivativeBounds;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddPathContinuityConstraints
            struct /* AddPathContinuityConstraints */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Enforces that for any two subsequent path segments within the
subgraph, the ``continuity_order`th path derivative at the end of the
first segment equals that of the start of the second segment.

Note that the constraints are on the control points of the derivatives
of r(s) and not q(t). This may result in discontinuities of the
trajectory return by `SolvePath()`` since the r(s) will get rescaled
by the duration h to yield q(t). ``NormalizeSegmentTimes()`` will
return r(s) with valid continuity.

Parameter ``continuity_order``:
    is the order of the continuity constraint.

Raises:
    RuntimeError if the continuity order is not equal or less than the
    order the subgraphs.

Raises:
    RuntimeError if the continuity order is less than one since path
    continuity is enforced by default.

Note:
    To enforce that the trajectory is of class C^k, you must call
    AddPathContinuityConstraint for each continuity_order 1 through k.)""";
            } AddPathContinuityConstraints;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddPathEnergyCost
            struct /* AddPathEnergyCost */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_1args_weight_matrix =
R"""(Similar to AddPathLengthCost in usage, but minimizes ∑ |weight_matrix
* (rᵢ₊₁ − rᵢ)|₂². In comparison to AddPathLength cost, this cost
encourages control points to be evenly spaced but may result in
greater number of regions and larger path length on the solution. It
is recommended to use this cost only with SolveConvexRestriction when
it becomes a quadratic cost for which some solvers show a better
performance.

Parameter ``weight_matrix``:
    is the relative weight of each component for the cost. The
    diagonal of the matrix is the weight for each dimension. The
    off-diagonal elements are the weight for the cross terms, which
    can be used to penalize diagonal movement.

Precondition:
    weight_matrix must be of size num_positions() x num_positions().)""";
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_1args_weight =
R"""(Similar to AddPathLengthCost in usage, but minimizes ∑ |(rᵢ₊₁ − rᵢ)|₂²
with weight being applied uniformly to all dimensions. In comparison
to AddPathLength cost, this cost encourages control points to be
evenly spaced but may result in greater number of regions and larger
path length on the solution. It is recommended to use this cost only
with SolveConvexRestriction when it becomes a quadratic cost for which
some solvers show a better performance.

Parameter ``weight``:
    is the relative weight of the cost.)""";
            } AddPathEnergyCost;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddPathLengthCost
            struct /* AddPathLengthCost */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_1args_weight_matrix =
R"""(Adds multiple L2Norm Costs on the upper bound of the path length.
Since we cannot directly compute the path length of a Bézier curve, we
minimize the upper bound of the path integral by minimizing the sum of
distances between control points. For Bézier curves, this is
equivalent to the sum of the L2Norm of the derivative control points
of the curve divided by the order.

Parameter ``weight_matrix``:
    is the relative weight of each component for the cost. The
    diagonal of the matrix is the weight for each dimension. The
    off-diagonal elements are the weight for the cross terms, which
    can be used to penalize diagonal movement.

Precondition:
    weight_matrix must be of size num_positions() x num_positions().)""";
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_1args_weight =
R"""(Adds multiple L2Norm Costs on the upper bound of the path length. We
upper bound the trajectory length by the sum of the distances between
control points. For Bézier curves, this is equivalent to the sum of
the L2Norm of the derivative control points of the curve divided by
the order.

Parameter ``weight``:
    is the relative weight of the cost.)""";
            } AddPathLengthCost;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddTimeCost
            struct /* AddTimeCost */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Adds a minimum time cost to all regions in the subgraph. The cost is
the sum of the time scaling variables.

Parameter ``weight``:
    is the relative weight of the cost.)""";
            } AddTimeCost;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddVelocityBounds
            struct /* AddVelocityBounds */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Adds a linear velocity constraint to the subgraph ``lb`` ≤ q̇(t) ≤
``ub``.

Parameter ``lb``:
    is the lower bound of the velocity.

Parameter ``ub``:
    is the upper bound of the velocity.

Raises:
    RuntimeError if subgraph order is zero, since the velocity is
    defined as the derivative of the Bézier curve.

Raises:
    RuntimeError if lb or ub are not of size num_positions().)""";
            } AddVelocityBounds;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddVertexConstraint
            struct /* AddVertexConstraint */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_e_use_in_transcription =
R"""(Adds an arbitrary user-defined constraint to every vertex in the
subgraph. The constraint should be defined using the placeholder
control point variables (obtained from vertex_control_points()) and
the placeholder time scaling variable (obtained from
vertex_duration()). This enables greater modeling freedom, but we
cannot guarantee a feasible solution for all possible constraints.

Raises:
    RuntimeError if any variables besides those from vertex_duration
    and vertex_control_points are used.

Constraints which do not support the perspective operation cannot be
used with Transcription∷kMIP or Transcription∷kRelaxation. Consider
providing an appropriate "convex surrogate" that is supported within
GraphOfConvexSets, or exclusively using the SolveConvexRestriction
method.)""";
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_binding_use_in_transcription =
R"""(Convenience overload of AddVertexConstraint to take in a
Binding<Constraint>.

Raises:
    RuntimeError if any variables besides those from vertex_duration
    and vertex_control_points are used.)""";
            } AddVertexConstraint;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::AddVertexCost
            struct /* AddVertexCost */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_e_use_in_transcription =
R"""(Adds an arbitrary user-defined cost to every vertex in the subgraph.
The cost should be defined using the placeholder control point
variables (obtained from vertex_control_points()) and the placeholder
time scaling variable (obtained from vertex_duration()). This enables
greater modeling freedom, but we cannot guarantee a feasible solution
for all possible costs.

Raises:
    RuntimeError if any variables besides those from vertex_duration
    and vertex_control_points are used.

Costs which do not support the perspective operation cannot be used
with Transcription∷kMIP or Transcription∷kRelaxation. Consider
providing an appropriate "convex surrogate" that is supported within
GraphOfConvexSets, or exclusively using the SolveConvexRestriction
method.)""";
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc_2args_binding_use_in_transcription =
R"""(Convenience overload of AddVertexCost to take in a Binding<Cost>.

Raises:
    RuntimeError if any variables besides those from vertex_duration
    and vertex_control_points are used.)""";
            } AddVertexCost;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::Edges
            struct /* Edges */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns constant reference to a vector of mutable pointers to the
edges.)""";
            } Edges;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::Subgraph
            struct /* ctor */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::Vertices
            struct /* Vertices */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns constant reference to a vector of mutable pointers to the
vertices stored in the subgraph. The order of the vertices is the same
as the order the regions were added.)""";
            } Vertices;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::edge_constituent_vertex_control_points
            struct /* edge_constituent_vertex_control_points */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns a pair of placeholder decision variables (not actually
declared as decision variables in the MathematicalProgram) associated
with the control points of the trajectory in two sets within this
subgraph that are connected by an internal edge. Each variable will be
of shape (num_positions(), order+1), where the ith column is the ith
control point. These variables will be substituted for real decision
variables in methods like AddEdgeCost and AddEdgeConstraint. Passing
this variable directly into objectives/constraints will result in an
error.)""";
            } edge_constituent_vertex_control_points;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::edge_constituent_vertex_durations
            struct /* edge_constituent_vertex_durations */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns a pair of placeholder decision variables (not actually
declared as decision variables in the MathematicalProgram) associated
with the time scaling of the trajectory in two sets within this
subgraph that are connected by an internal edge. These variables will
be substituted for real decision variables in methods like AddEdgeCost
and AddEdgeConstraint. Passing this variable directly into
objectives/constraints will result in an error.)""";
            } edge_constituent_vertex_durations;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::name
            struct /* name */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc = R"""(Returns the name of the subgraph.)""";
            } name;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::order
            struct /* order */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns the order of the Bézier trajectory within the region.)""";
            } order;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::regions
            struct /* regions */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns the regions associated with this subgraph before the
CartesianProduct.)""";
            } regions;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::size
            struct /* size */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns the number of vertices in the subgraph.)""";
            } size;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::vertex_control_points
            struct /* vertex_control_points */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns a placeholder decision variable (not actually declared as a
decision variable in the MathematicalProgram) associated with the
control points of the trajectory in a set within this subgraph. The
variable will be of shape (num_positions(), order+1), where the ith
column is the ith control point. This variable will be substituted for
real decision variables in methods like AddVertexCost and
AddVertexConstraint. Passing this variable directly into
objectives/constraints will result in an error.)""";
            } vertex_control_points;
            // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph::vertex_duration
            struct /* vertex_duration */ {
              // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
              const char* doc =
R"""(Returns a placeholder decision variable (not actually declared as a
decision variable in the MathematicalProgram) associated with the time
scaling of the trajectory in a set within this subgraph. This variable
will be substituted for real decision variables in methods like
AddVertexCost and AddVertexConstraint. Passing this variable directly
into objectives/constraints will result in an error.)""";
            } vertex_duration;
          } Subgraph;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::UnwrapToContinuousTrajectory
          struct /* UnwrapToContinuousTrajectory */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Unwraps a trajectory with continuous revolute joints into a continuous
trajectory in the Euclidean space. Trajectories produced by
GcsTrajectoryOptimization for robotic systems with continuous revolute
joints may include apparent discontinuities, where a multiple of 2π is
instantaneously added to a joint value at the boundary between two
adjacent segments of the trajectory. This function removes such
discontinuities by adding or subtracting the appropriate multiple of
2π, "unwrapping" the trajectory into a continuous representation
suitable for downstream tasks that do not take the joint wraparound
into account.

Parameter ``gcs_trajectory``:
    The trajectory to unwrap.

Parameter ``continuous_revolute_joints``:
    The indices of the continuous revolute joints.

Parameter ``tol``:
    The numerical tolerance used to determine if two subsequent
    segments start and end at the same value modulo 2π for continuous
    revolute joints.

Parameter ``starting_rounds``:
    A vector of integers that sets the starting rounds for each
    continuous revolute joint. Given integer k for the starting_round
    of a joint, its initial position will be wrapped into [2πk ,
    2π(k+1)). If the starting rounds are not provided, the initial
    position of ``trajectory`` will be unchanged.

Returns:
    an unwrapped (continuous in Euclidean space) CompositeTrajectory.

Raises:
    RuntimeError if
    starting_rounds.size()!=continuous_revolute_joints.size().

Raises:
    RuntimeError if continuous_revolute_joints contain repeated
    indices and/or indices outside the range [0,
    gcs_trajectory.rows()).

Raises:
    RuntimeError if the gcs_trajectory is not continuous on the
    manifold defined by the continuous_revolute_joints, i.e., the
    shift between two consecutive segments is not an integer multiple
    of 2π (within a tolerance of ``tol`` radians).

Raises:
    RuntimeError if all the segments are not of type BezierCurve.
    Other types are not supported yet. Note that currently the output
    of GcsTrajectoryOptimization∷SolvePath() is a CompositeTrajectory
    of BezierCurves.)""";
          } UnwrapToContinuousTrajectory;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::continuous_revolute_joints
          struct /* continuous_revolute_joints */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Returns a list of indices corresponding to continuous revolute joints.)""";
          } continuous_revolute_joints;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::graph_of_convex_sets
          struct /* graph_of_convex_sets */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Getter for the underlying GraphOfConvexSets. This is intended
primarily for inspecting the resulting programs.)""";
          } graph_of_convex_sets;
          // Symbol: drake::planning::trajectory_optimization::GcsTrajectoryOptimization::num_positions
          struct /* num_positions */ {
            // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
            const char* doc =
R"""(Returns the number of position variables.)""";
          } num_positions;
        } GcsTrajectoryOptimization;
        // Symbol: drake::planning::trajectory_optimization::GetContinuousRevoluteJointIndices
        struct /* GetContinuousRevoluteJointIndices */ {
          // Source: drake/planning/trajectory_optimization/gcs_trajectory_optimization.h
          const char* doc =
R"""(Returns a list of indices in the plant's generalized positions which
correspond to a continuous revolute joint (a revolute joint with no
joint limits). This includes UniversalJoint, and the revolute
component of PlanarJoint and RpyFloatingJoint.)""";
        } GetContinuousRevoluteJointIndices;
        // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization
        struct /* KinematicTrajectoryOptimization */ {
          // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
          const char* doc =
R"""(Optimizes a trajectory, q(t) subject to costs and constraints on the
trajectory and its derivatives. This is accomplished using a ``path``,
r(s), represented as a BsplineTrajectory on the interval s∈[0,1], and
a separate duration, T, which maps [0,1] => [0,T].

The q(t) trajectory is commonly associated with, for instance, the
generalized positions of a MultibodyPlant by adding multibody costs
and constraints; in this case take note that the velocities in this
optimization are q̇(t), not v(t).

Use solvers∷Solve to solve the problem. A typical use case could look
like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    KinematicTrajectoryOptimization trajopt(2, 10);
    // add costs and constraints
    trajopt.SetInitialGuess(...);
    auto result = solvers∷Solve(trajopt.prog());
    auto traj = trajopt.ReconstructTrajectory(result);

.. raw:: html

    </details>

When possible this class attempts to formulate convex forms of the
costs and constraints.)""";
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddAccelerationBounds
          struct /* AddAccelerationBounds */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds generic (nonlinear) constraints to enforce the upper and lower
bounds to the acceleration trajectory, q̈(t). By leveraging the convex
hull property of B-splines, these bounds are applied at the
(derivative) control points, but will be respected for all times, t ∈
[0,T]. Note that this does NOT directly constrain r̈(s).

Returns:
    A vector of bindings with interleaved lower and then upper bounds,
    constraining all of the control points for one element of q̈ (e.g.
    the acceleration of the ith joint at all times). However, this
    specific interpretation of these constraints may change without
    deprecation.)""";
          } AddAccelerationBounds;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddDurationConstraint
          struct /* AddDurationConstraint */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds bounding box constraints for upper and lower bounds on the
duration of the trajectory.)""";
          } AddDurationConstraint;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddDurationCost
          struct /* AddDurationCost */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds a linear cost on the duration of the trajectory.)""";
          } AddDurationCost;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddEffortBoundsAtNormalizedTimes
          struct /* AddEffortBoundsAtNormalizedTimes */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds generic (nonlinear) constraints to enforce the effort limits
defined in the plant at a sequence of normalized times, ``s``:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    B lb ≤ M(q)v̇ + C(q, v)v - τ_g(q) - τ_app ≤ B ub

.. raw:: html

    </details>

where q, v, and v̇ are evaluated at s. B is the plant's actuation
matrix, and M, C, τ_g, and τ_app are the plant's mass matrix, Coriolis
force, gravity, and applied force, respectively. ``ub`` and ``lb`` are
the upper and lower effort bounds, respectively; if they are not
provided then plant.GetEffortLowerLimits() and
plant.GetEffortUpperLimits() are used.

Pass ``plant_context`` if you have non-default parameters in the
context. Note that there are no lifetime requirements on ``plant`` nor
``plant_context``.

Note that the convex hull property of the B-splines is not guaranteed
to hold here -- effort limits maybe be violated away from the
normalized times ``s``.

Precondition:
    plant.is_finalized()

Precondition:
    plant.num_positions() == num_positions()

Precondition:
    plant.IsVelocityEqualToQDot() == true

Precondition:
    s[i] ∈ [0, 1] for all i

Precondition:
    B lb ≤ B ub

Returns:
    A vector of bindings with one effort limit constraint for each
    ``s``.)""";
          } AddEffortBoundsAtNormalizedTimes;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddJerkBounds
          struct /* AddJerkBounds */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds generic (nonlinear) constraints to enforce the upper and lower
bounds to the jerk trajectory, d³qdt³(t). By leveraging the convex
hull property of B-splines, these bounds are applied at the
(derivative) control points, but will be respected for all times, t ∈
[0,T]. Note that this does NOT directly constrain d³rds³(s).

Returns:
    A vector of bindings with interleaved lower and then upper bounds,
    constraining all of the control points for one element of d³qdt³
    (e.g. the jerk of the ith joint at all times). However, this
    specific interpretation of these constraints may change without
    deprecation.)""";
          } AddJerkBounds;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddPathAccelerationConstraint
          struct /* AddPathAccelerationConstraint */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds a linear constraint on the second derivative of the path, ``lb``
≤ r̈(s) ≤ ``ub``. Note that this does NOT directly constrain q̈(t).

Precondition:
    0 <= ``s`` <= 1.)""";
          } AddPathAccelerationConstraint;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddPathEnergyCost
          struct /* AddPathEnergyCost */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds a convex quadratic cost on an upper bound on the energy of the
path, ∫₀¹ |ṙ(s)|₂² ds, by summing the squared distance between the
path control points. In the limit of infinitely many control points,
minimizers for AddPathLengthCost and AddPathEnergyCost will follow the
same path, but potentially with different timing. They may have
different values if additional costs and constraints are imposed. This
cost yields simpler gradients than AddPathLengthCost, and biases the
control points towards being evenly spaced.

Returns:
    A vector of bindings with the ith element adding a cost to the ith
    control point of the velocity trajectory. However, this specific
    interpretation of these constraints may change without
    deprecation.)""";
          } AddPathEnergyCost;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddPathLengthCost
          struct /* AddPathLengthCost */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds a cost on an upper bound of the length of the path, ∫₀ᵀ |q̇(t)|₂
dt, or equivalently ∫₀¹ |ṙ(s)|₂ ds, by summing the distance between
the path control points. If ``use_conic_constraint = false``, then
costs are added via MathematicalProgram∷AddL2NormCost; otherwise they
are added via MathematicalProgram∷AddL2NormCostUsingConicConstraint.

Returns:
    A vector of bindings with the ith element adding a cost to the ith
    control point of the velocity trajectory. However, this specific
    interpretation of these constraints may change without
    deprecation.)""";
          } AddPathLengthCost;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddPathPositionConstraint
          struct /* AddPathPositionConstraint */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc_3args =
R"""(Adds a linear constraint on the value of the path, ``lb`` ≤ r(s) ≤
``ub``.

Precondition:
    0 <= ``s`` <= 1.)""";
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc_2args =
R"""(Adds a (generic) constraint on path. The constraint will be evaluated
as if it is bound with variables corresponding to ``r(s)``.

Precondition:
    constraint.num_vars() == num_positions()

Precondition:
    0 <= ``s`` <= 1.)""";
          } AddPathPositionConstraint;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddPathVelocityConstraint
          struct /* AddPathVelocityConstraint */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds a linear constraint on the derivative of the path, ``lb`` ≤ ṙ(s)
≤ ``ub``. Note that this does NOT directly constrain q̇(t).

Precondition:
    0 <= ``s`` <= 1.)""";
          } AddPathVelocityConstraint;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddPositionBounds
          struct /* AddPositionBounds */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds bounding box constraints to enforce upper and lower bounds on the
positions trajectory, q(t). These bounds will be respected at all
times, t∈[0,T]. This also implies the constraints are satisfied for
r(s), for all s∈[0,1].

Returns:
    A vector of bindings with the ith element adding a constraint to
    the ith control point. However, this specific interpretation of
    these constraints may change without deprecation.)""";
          } AddPositionBounds;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddVelocityBounds
          struct /* AddVelocityBounds */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Adds linear constraints to enforce upper and lower bounds on the
velocity trajectory, q̇(t). By leveraging the convex hull property of
B-splines, these bounds are applied at the (derivative) control
points, but will be respected for all times, t ∈ [0,T]. Note this does
NOT directly constrain ṙ(s).

Returns:
    A vector of bindings with interleaved lower and then upper bounds,
    constraining all of the control points for one element of q̇ (e.g.
    the velocity of the ith joint at all times). However, this
    specific interpretation of these constraints may change without
    deprecation..)""";
          } AddVelocityBounds;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::AddVelocityConstraintAtNormalizedTime
          struct /* AddVelocityConstraintAtNormalizedTime */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc_2args_constraint_s =
R"""(Adds a (generic) constraint on trajectory velocity ``q̇(t)``,
evaluated at ``s``. The constraint will be evaluated as if it is bound
with variables corresponding to ``[q(T*s), q̇(T*s)]``.

This is a potentially confusing mix of ``s`` and ``t``, but it is
important in practice. For instance if you want to constrain the true
(trajectory) velocity at the final time, one would naturally want to
write AddVelocityConstraint(constraint, s=1).

This method should be compared with AddPathVelocityConstraint, which
only constrains ṙ(s) because it does not reason about the time
scaling, T. However, AddPathVelocityConstraint adds convex
constraints, whereas this method adds nonconvex generic constraints.

Precondition:
    constraint.num_vars() == num_positions()

Precondition:
    0 <= ``s`` <= 1.)""";
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc_2args_binding_s =
R"""(Adds a linear constraint on some (or all) of the placeholder variables
qdot, evaluated at a normalized time s.

Precondition:
    binding Can only associate with qdot().

Precondition:
    0 <= ``s`` <= 1.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cpp}
    Binding<LinearConstraint> b(LinearConstraint(A, b), trajopt.qdot());
    trajopt.AddVelocityConstraintAtNormalizedTime(b, 0);

.. raw:: html

    </details>)""";
          } AddVelocityConstraintAtNormalizedTime;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::KinematicTrajectoryOptimization
          struct /* ctor */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc_4args =
R"""(Constructs an optimization problem for a position trajectory
represented as a B-spline. The initial guess is the zero trajectory
over the time interval [0, T].

Parameter ``num_positions``:
    The number of rows in the B-spline.

Parameter ``num_control_points``:
    The number of B-spline control points.

Parameter ``spline_order``:
    The order of the B-spline.

Parameter ``duration``:
    The duration (in seconds) of the initial guess.)""";
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc_1args =
R"""(Constructs an optimization problem for a trajectory represented by a
B-spline with the same order and number of control points as
``trajectory``. Additionally sets ``trajectory`` as the initial guess
for the optimization.)""";
          } ctor;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::ReconstructTrajectory
          struct /* ReconstructTrajectory */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Returns the trajectory q(t) from the ``result`` of solving ``prog()``.)""";
          } ReconstructTrajectory;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::SetInitialGuess
          struct /* SetInitialGuess */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Sets the initial guess for the path and duration to match
``trajectory``.

Precondition:
    trajectory.rows() == num_positions()

Precondition:
    trajectory.columns() == 1)""";
          } SetInitialGuess;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::basis
          struct /* basis */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Returns the basis used to represent the path, r(s), over s∈[0,1].)""";
          } basis;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::control_points
          struct /* control_points */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Returns the control points defining the path as an M-by-N matrix,
where M is the number of positions and N is the number of control
points.)""";
          } control_points;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::duration
          struct /* duration */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Returns the decision variable defining the time duration of the
trajectory.)""";
          } duration;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::get_mutable_prog
          struct /* get_mutable_prog */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Getter for a mutable pointer to the optimization program.)""";
          } get_mutable_prog;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::num_control_points
          struct /* num_control_points */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Returns the number of control points used for the path.)""";
          } num_control_points;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::num_positions
          struct /* num_positions */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Returns the number of position variables.)""";
          } num_positions;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::prog
          struct /* prog */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc = R"""(Getter for the optimization program.)""";
          } prog;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::q
          struct /* q */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Returns the placeholder variable for generalized position q. Note
these are NOT decision variables in the MathematicalProgram. These
variables will be substituted for the real decision variables at
particular times. Passing these variables directily into
objective/constraints for the MathematicalProgram will result in an
error.)""";
          } q;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::qddot
          struct /* qddot */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Returns the placeholder variable for the second time derivative of
generalized position. Note these are NOT decision variables in the
MathematicalProgram. These variables will be substituted for the real
decision variables at particular times. Passing these variables
directily into objective/constraints for the MathematicalProgram will
result in an error.)""";
          } qddot;
          // Symbol: drake::planning::trajectory_optimization::KinematicTrajectoryOptimization::qdot
          struct /* qdot */ {
            // Source: drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h
            const char* doc =
R"""(Returns the placeholder variable for the time derivative of
generalized position. Note these are NOT decision variables in the
MathematicalProgram. These variables will be substituted for the real
decision variables at particular times. Passing these variables
directily into objective/constraints for the MathematicalProgram will
result in an error.)""";
          } qdot;
        } KinematicTrajectoryOptimization;
        // Symbol: drake::planning::trajectory_optimization::MidPointIntegrationConstraint
        struct /* MidPointIntegrationConstraint */ {
          // Source: drake/planning/trajectory_optimization/integration_constraint.h
          const char* doc =
R"""(Implements the midpoint integration

(ẋₗ + ẋᵣ)/2 * dt = xᵣ - xₗ where the bounded variables are (xᵣ, xₗ,
ẋᵣ, ẋₗ, dt))""";
          // Symbol: drake::planning::trajectory_optimization::MidPointIntegrationConstraint::ComposeX
          struct /* ComposeX */ {
            // Source: drake/planning/trajectory_optimization/integration_constraint.h
            const char* doc =
R"""(Compose x for the Eval input from individual variables.)""";
          } ComposeX;
          // Symbol: drake::planning::trajectory_optimization::MidPointIntegrationConstraint::MidPointIntegrationConstraint
          struct /* ctor */ {
            // Source: drake/planning/trajectory_optimization/integration_constraint.h
            const char* doc = R"""()""";
          } ctor;
        } MidPointIntegrationConstraint;
        // Symbol: drake::planning::trajectory_optimization::MultipleShooting
        struct /* MultipleShooting */ {
          // Source: drake/planning/trajectory_optimization/multiple_shooting.h
          const char* doc =
R"""(MultipleShooting is an abstract class for trajectory optimization that
creates decision variables for inputs, states, and (optionally) sample
times along the trajectory, then provides a number of methods for
working with those decision variables.

MultipleShooting classes add decision variables, costs, and
constraints to a MathematicalProgram. You can retrieve that program
using prog(), and add additional variables, costs, and constraints
using the MathematicalProgram interface directly.

Subclasses must implement the abstract methods: DoAddRunningCost()
ReconstructInputTrajectory() ReconstructStateTrajectory() using all of
the correct interpolation schemes for the specific transcription
method, and should add the constraints to impose the System% dynamics
in their constructor.

This class assumes that there are a fixed number (N) time
steps/samples, and that the trajectory is discretized into time steps
h (N-1 of these), state x (N of these), and control input u (N of
these).)""";
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::AddCompleteTrajectoryCallback
          struct /* AddCompleteTrajectoryCallback */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Adds a callback method to visualize intermediate results of all
variables used in the trajectory optimization. The callback should be
of the form MyVisualization(sample_times, states, inputs, values),
where sample_times is an N-by-1 VectorXd of sample times, states is a
num_states-by-N MatrixXd of the current (intermediate) state
trajectory at the break points, inputs is a num_inputs-by-N MatrixXd
of the current (intermediate) input trajectory at the break points and
values is a vector of num_rows-by-N MatrixXds of the current
(intermediate) extra sequential variables specified by ``names`` at
the break points.

Note:
    Just like other costs/constraints, not all solvers support
    callbacks. Adding a callback here will force
    MathematicalProgram∷Solve to select a solver that support
    callbacks. For instance, adding a visualization callback to a
    quadratic programming problem may result in using a nonlinear
    programming solver as the default solver.)""";
          } AddCompleteTrajectoryCallback;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::AddConstraintToAllKnotPoints
          struct /* AddConstraintToAllKnotPoints */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_shared_ptr =
R"""(Adds a constraint to all breakpoints, where any instances in ``vars``
of time(), state(), and/or input() placeholder variables, as well as
placeholder variables returned by calls to NewSequentialVariable(),
are substituted with the relevant variables for each time index.

Returns:
    A vector of the bindings added to each knot point.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Adds a constraint to all breakpoints, where any instances of time(),
state(), and/or input() placeholder variables, as well as placeholder
variables returned by calls to NewSequentialVariable(), are
substituted with the relevant variables for each time index.

Returns:
    A vector of the bindings added to each knot point.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_formulas =
R"""(Variant of AddConstraintToAllKnotPoints that accepts a vector of
formulas.)""";
          } AddConstraintToAllKnotPoints;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::AddDurationBounds
          struct /* AddDurationBounds */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Adds a constraint on the total duration of the trajectory.

Parameter ``lower_bound``:
    A scalar double lower bound.

Parameter ``upper_bound``:
    A scalar double upper bound.

Returns:
    The constraint enforcing the duration bounds.

Raises:
    RuntimeError if time steps are not declared as decision variables.)""";
          } AddDurationBounds;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::AddEqualTimeIntervalsConstraints
          struct /* AddEqualTimeIntervalsConstraints */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Adds constraints to enforce that all time steps have equal duration.

Returns:
    A vector of constraints enforcing all time intervals are equal.

Raises:
    RuntimeError if time steps are not declared as decision variables.)""";
          } AddEqualTimeIntervalsConstraints;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::AddFinalCost
          struct /* AddFinalCost */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_1args_e =
R"""(Adds a cost to the final time, of the form

.. math:: cost = e(t,x,u),

where any instances of time(), state(), and/or input() placeholder
variables, as well as placeholder variables returned by calls to
NewSequentialVariable(), are substituted with the relevant variables
for the final time index.

Returns:
    The final cost added to the problem.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_1args_matrix =
R"""(Adds support for passing in a (scalar) matrix Expression, which is a
common output of most symbolic linear algebra operations.

Returns:
    The final cost added to the problem.

Note:
    Derived classes will need to type using
    MultipleShooting∷AddFinalCost; to "unhide" this method.)""";
          } AddFinalCost;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::AddInputTrajectoryCallback
          struct /* AddInputTrajectoryCallback */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Adds a callback method to visualize intermediate results of input
variables used in the trajectory optimization. The callback should be
of the form MyVisualization(sample_times, values), where breaks is a
N-by-1 VectorXd of sample times, and values is a num_inputs-by-N
MatrixXd representing the current (intermediate) value of the input
trajectory at the break points in each column.

Note:
    Just like other costs/constraints, not all solvers support
    callbacks. Adding a callback here will force
    MathematicalProgram∷Solve to select a solver that support
    callbacks. For instance, adding a visualization callback to a
    quadratic programming problem may result in using a nonlinear
    programming solver as the default solver.)""";
          } AddInputTrajectoryCallback;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::AddRunningCost
          struct /* AddRunningCost */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_1args_g =
R"""(Adds an integrated cost to all time steps, of the form

.. math:: cost = \int_0^T g(t,x,u) dt,

where any instances of time(), state(), and/or input() placeholder
variables, as well as placeholder variables returned by calls to
NewSequentialVariable(), are substituted with the relevant variables
for each time index. The particular integration scheme is determined
by the derived class implementation.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_1args_constEigenMatrixBase =
R"""(Adds support for passing in a (scalar) matrix Expression, which is a
common output of most symbolic linear algebra operations.)""";
          } AddRunningCost;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::AddStateTrajectoryCallback
          struct /* AddStateTrajectoryCallback */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Adds a callback method to visualize intermediate results of state
variables used in the trajectory optimization. The callback should be
of the form MyVisualization(sample_times, values), where sample_times
is a N-by-1 VectorXd of sample times, and values is a num_states-by-N
MatrixXd representing the current (intermediate) value of the state
trajectory at the break points in each column.

Note:
    Just like other costs/constraints, not all solvers support
    callbacks. Adding a callback here will force
    MathematicalProgram∷Solve to select a solver that support
    callbacks. For instance, adding a visualization callback to a
    quadratic programming problem may result in using a nonlinear
    programming solver as the default solver.)""";
          } AddStateTrajectoryCallback;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::AddTimeIntervalBounds
          struct /* AddTimeIntervalBounds */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Adds bounds on all time intervals.

Parameter ``lower_bound``:
    A scalar double lower bound.

Parameter ``upper_bound``:
    A scalar double upper bound.

Returns:
    The bounding box constraint enforcing time interval bounds.

Raises:
    RuntimeError if time steps are not declared as decision variables.)""";
          } AddTimeIntervalBounds;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::CompleteTrajectoryCallback
          struct /* CompleteTrajectoryCallback */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } CompleteTrajectoryCallback;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::GetInputSamples
          struct /* GetInputSamples */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns a matrix containing the input values (arranged in columns) at
each breakpoint at the solution.)""";
          } GetInputSamples;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::GetSampleTimes
          struct /* GetSampleTimes */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_1args_h_var_values =
R"""(Returns a vector containing the elapsed time at each breakpoint.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_1args_result =
R"""(Returns a vector containing the elapsed time at each breakpoint at the
solution.)""";
          } GetSampleTimes;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::GetSequentialVariable
          struct /* GetSequentialVariable */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns the decision variables associated with the sequential variable
``name``.)""";
          } GetSequentialVariable;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::GetSequentialVariableAtIndex
          struct /* GetSequentialVariableAtIndex */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns the decision variables associated with the sequential variable
``name`` at time index ``index``.

See also:
    NewSequentialVariable().)""";
          } GetSequentialVariableAtIndex;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::GetSequentialVariableSamples
          struct /* GetSequentialVariableSamples */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns a matrix containing the sequential variable values (arranged
in columns) at each breakpoint at the solution.

Parameter ``name``:
    The name of sequential variable to get the results for. Must
    correspond to an already added sequential variable.)""";
          } GetSequentialVariableSamples;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::GetStateSamples
          struct /* GetStateSamples */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns a matrix containing the state values (arranged in columns) at
each breakpoint at the solution.)""";
          } GetStateSamples;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::MultipleShooting
          struct /* ctor */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_5args_num_inputs_num_states_num_time_samples_fixed_time_step_prog =
R"""(Constructs a MultipleShooting instance with fixed sample times. It
creates new placeholder variables for input and state.

Parameter ``num_inputs``:
    Number of inputs at each sample point.

Parameter ``num_states``:
    Number of states at each sample point.

Parameter ``num_time_samples``:
    Number of time samples.

Parameter ``fixed_time_step``:
    The spacing between sample times.

Parameter ``prog``:
    (optional). If non-null, then additional decision variables,
    costs, and constraints will be added into the existing
    MathematicalProgram. This can be useful for, e.g., combining
    multiple trajectory optimizations into a single program, coupled
    by a few constraints. If nullptr, then a new MathematicalProgram
    will be allocated.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_5args_input_state_num_time_samples_fixed_time_step_prog =
R"""(Constructs a MultipleShooting instance with fixed sample times. It
uses the provided ``input`` and ``state`` as placeholders instead of
creating new placeholder variables for them.

Parameter ``input``:
    Placeholder variables for input.

Parameter ``state``:
    Placeholder variables for state.

Parameter ``num_time_samples``:
    Number of time samples.

Parameter ``fixed_time_step``:
    The spacing between sample times.

Parameter ``prog``:
    (optional). If non-null, then additional decision variables,
    costs, and constraints will be added into the existing
    MathematicalProgram. This can be useful for, e.g., combining
    multiple trajectory optimizations into a single program, coupled
    by a few constraints. If nullptr, then a new MathematicalProgram
    will be allocated.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_6args_num_inputs_num_states_num_time_samples_minimum_time_step_maximum_time_step_prog =
R"""(Constructs a MultipleShooting instance with sample times as decision
variables. It creates new placeholder variables for input, state, and
time.

Parameter ``num_inputs``:
    Number of inputs at each sample point.

Parameter ``num_states``:
    Number of states at each sample point.

Parameter ``num_time_samples``:
    Number of time samples.

Parameter ``minimum_time_step``:
    Minimum spacing between sample times.

Parameter ``maximum_time_step``:
    Maximum spacing between sample times.

Parameter ``prog``:
    (optional). If non-null, then additional decision variables,
    costs, and constraints will be added into the existing
    MathematicalProgram. This can be useful for, e.g., combining
    multiple trajectory optimizations into a single program, coupled
    by a few constraints. If nullptr, then a new MathematicalProgram
    will be allocated.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_7args_input_state_time_num_time_samples_minimum_time_step_maximum_time_step_prog =
R"""(Constructs a MultipleShooting instance with sample times as decision
variables. It uses the provided ``input``, `state`, and ``time`` as
placeholders instead of creating new placeholder variables for them.

Parameter ``input``:
    Placeholder variables for input.

Parameter ``state``:
    Placeholder variables for state.

Parameter ``time``:
    Placeholder variable for time.

Parameter ``num_time_samples``:
    Number of time samples.

Parameter ``minimum_time_step``:
    Minimum spacing between sample times.

Parameter ``maximum_time_step``:
    Maximum spacing between sample times.

Parameter ``prog``:
    (optional). If non-null, then additional decision variables,
    costs, and constraints will be added into the existing
    MathematicalProgram. This can be useful for, e.g., combining
    multiple trajectory optimizations into a single program, coupled
    by a few constraints. If nullptr, then a new MathematicalProgram
    will be allocated.)""";
          } ctor;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::N
          struct /* N */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } N;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::NewSequentialVariable
          struct /* NewSequentialVariable */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Adds a sequential variable (a variable that has associated decision
variables for each time index) to the optimization problem and returns
a placeholder variable (not actually declared as a decision variable
in the MathematicalProgram). This variable will be substituted for
real decision variables at particular times in methods like
AddRunningCost(). Passing this variable directly into
objectives/constraints for the parent classes will result in an error.)""";
          } NewSequentialVariable;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::ReconstructInputTrajectory
          struct /* ReconstructInputTrajectory */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Get the input trajectory at the solution as a PiecewisePolynomial. The
order of the trajectory will be determined by the integrator used in
the dynamic constraints. Requires that the system has at least one
input port.)""";
          } ReconstructInputTrajectory;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::ReconstructStateTrajectory
          struct /* ReconstructStateTrajectory */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Get the state trajectory at the solution as a PiecewisePolynomial. The
order of the trajectory will be determined by the integrator used in
the dynamic constraints.)""";
          } ReconstructStateTrajectory;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::SetInitialTrajectory
          struct /* SetInitialTrajectory */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } SetInitialTrajectory;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::SubstitutePlaceholderVariables
          struct /* SubstitutePlaceholderVariables */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Replaces e.g. placeholder_x_var_ with x_vars_ at time interval
``interval_index``, for all placeholder variables.)""";
          } SubstitutePlaceholderVariables;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::TrajectoryCallback
          struct /* TrajectoryCallback */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } TrajectoryCallback;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::final_state
          struct /* final_state */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns the decision variables associated with the state, x, at the
final time index.)""";
          } final_state;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::fixed_time_step
          struct /* fixed_time_step */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } fixed_time_step;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::h_vars
          struct /* h_vars */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } h_vars;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::initial_state
          struct /* initial_state */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns the decision variables associated with the state, x, at the
initial time index.)""";
          } initial_state;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::input
          struct /* input */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_0args =
R"""(Returns placeholder decision variables (not actually declared as
decision variables in the MathematicalProgram) associated with the
input, u, but with the time-index undetermined. These variables will
be substituted for real decision variables at particular times in
methods like AddRunningCost. Passing these variables directly into
objectives/constraints for the parent classes will result in an error.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_1args =
R"""(Returns the decision variables associated with the input, u, at time
index ``index``.)""";
          } input;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::num_inputs
          struct /* num_inputs */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } num_inputs;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::num_states
          struct /* num_states */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } num_states;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::prog
          struct /* prog */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns a reference to the MathematicalProgram associated with the
trajectory optimization problem.)""";
          } prog;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::state
          struct /* state */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_0args =
R"""(Returns placeholder decision variables (not actually declared as
decision variables in the MathematicalProgram) associated with the
state, x, but with the time-index undetermined. These variables will
be substituted for real decision variables at particular times in
methods like AddRunningCost. Passing these variables directly into
objectives/constraints for the parent classes will result in an error.)""";
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc_1args =
R"""(Returns the decision variables associated with the state, x, at time
index ``index``.)""";
          } state;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::time
          struct /* time */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns a placeholder decision variable (not actually declared as a
decision variable in the MathematicalProgram) associated with the
time, t. This variable will be substituted for real decision variables
at particular times in methods like AddRunningCost. Passing this
variable directly into objectives/constraints for the parent classes
will result in an error.)""";
          } time;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::time_step
          struct /* time_step */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc =
R"""(Returns the decision variable associated with the time step, h, at
time index ``index``.

Raises:
    RuntimeError if time steps are not declared as decision variables.)""";
          } time_step;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::time_steps_are_decision_variables
          struct /* time_steps_are_decision_variables */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } time_steps_are_decision_variables;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::u_vars
          struct /* u_vars */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } u_vars;
          // Symbol: drake::planning::trajectory_optimization::MultipleShooting::x_vars
          struct /* x_vars */ {
            // Source: drake/planning/trajectory_optimization/multiple_shooting.h
            const char* doc = R"""()""";
          } x_vars;
        } MultipleShooting;
        // Symbol: drake::planning::trajectory_optimization::TimeStep
        struct /* TimeStep */ {
          // Source: drake/planning/trajectory_optimization/direct_transcription.h
          const char* doc = R"""()""";
          // Symbol: drake::planning::trajectory_optimization::TimeStep::TimeStep
          struct /* ctor */ {
            // Source: drake/planning/trajectory_optimization/direct_transcription.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::planning::trajectory_optimization::TimeStep::value
          struct /* value */ {
            // Source: drake/planning/trajectory_optimization/direct_transcription.h
            const char* doc = R"""()""";
          } value;
        } TimeStep;
      } trajectory_optimization;
    } planning;
  } drake;
} pydrake_doc_planning_trajectory_optimization;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
