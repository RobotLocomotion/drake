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

// #include "drake/planning/body_shape_description.h"
// #include "drake/planning/collision_avoidance.h"
// #include "drake/planning/collision_checker.h"
// #include "drake/planning/collision_checker_context.h"
// #include "drake/planning/collision_checker_params.h"
// #include "drake/planning/distance_and_interpolation_provider.h"
// #include "drake/planning/dof_mask.h"
// #include "drake/planning/edge_measure.h"
// #include "drake/planning/joint_limits.h"
// #include "drake/planning/linear_distance_and_interpolation_provider.h"
// #include "drake/planning/robot_clearance.h"
// #include "drake/planning/robot_collision_type.h"
// #include "drake/planning/robot_diagram.h"
// #include "drake/planning/robot_diagram_builder.h"
// #include "drake/planning/scene_graph_collision_checker.h"
// #include "drake/planning/unimplemented_collision_checker.h"
// #include "drake/planning/visibility_graph.h"

// Symbol: pydrake_doc_planning
constexpr struct /* pydrake_doc_planning */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::planning
    struct /* planning */ {
      // Symbol: drake::planning::BodyShapeDescription
      struct /* BodyShapeDescription */ {
        // Source: drake/planning/body_shape_description.h
        const char* doc =
R"""(BodyShapeDescription captures all the information necessary to
describe a SceneGraph collision shape associated with a MultibodyPlant
Body: a shape S, the MultibodyPlant body B (identified by model
instance and body names), and the rigid pose of the shape S relative
to the body B, X_BS.

Most clients should use the factory method MakeBodyShapeDescription()
to construct a valid BodyShapeDescription; it will extract and verify
the correct information from a multibody plant and its context.

When moved-from, this object models a "null" description and all of
the getter functions will throw.)""";
        // Symbol: drake::planning::BodyShapeDescription::BodyShapeDescription
        struct /* ctor */ {
          // Source: drake/planning/body_shape_description.h
          const char* doc =
R"""(Constructs a description with the given attributes. Does not check or
enforce correctness; callers are responsible for providing consistent
input.)""";
        } ctor;
        // Symbol: drake::planning::BodyShapeDescription::body_name
        struct /* body_name */ {
          // Source: drake/planning/body_shape_description.h
          const char* doc =
R"""(Returns:
    the body name passed at construction.)""";
        } body_name;
        // Symbol: drake::planning::BodyShapeDescription::model_instance_name
        struct /* model_instance_name */ {
          // Source: drake/planning/body_shape_description.h
          const char* doc =
R"""(Returns:
    the model instance name passed at construction.)""";
        } model_instance_name;
        // Symbol: drake::planning::BodyShapeDescription::pose_in_body
        struct /* pose_in_body */ {
          // Source: drake/planning/body_shape_description.h
          const char* doc =
R"""(Returns ``X_BS``:
    The pose passed at construction.)""";
        } pose_in_body;
        // Symbol: drake::planning::BodyShapeDescription::shape
        struct /* shape */ {
          // Source: drake/planning/body_shape_description.h
          const char* doc =
R"""(Returns:
    the shape passed at construction.)""";
        } shape;
      } BodyShapeDescription;
      // Symbol: drake::planning::CollisionChecker
      struct /* CollisionChecker */ {
        // Source: drake/planning/collision_checker.h
        const char* doc =
R"""(Interface for collision checkers to use.

This interface builds on the basic multi-threading idiom of Drake: one
context per thread. It offers two models to achieve multi-threaded
parallel collision checking:

- using thread pools (e.g. OpenMP or similar) and "implicit contexts" managed
by this object
- using arbitrary threads and "explicit contexts" created by this object

<h5>Implicit Context Parallelism</h5>

Many methods of this class aren't designed for entry from arbitrary
threads (e.g. from std∷async threads), but rather are designed for use
with a main thread and various thread-pool-parallel operations
achieved by using directives like ``omp parallel``. To support this
usage, the base class ``AllocateContexts()`` protected method
establishes a pool of contexts to support the implicit context
parallelism specified in the constructor. (Note: if the collision
checker declares that parallel checking is not supported, only one
implict context will be allocated). ``AllocateContexts()`` must be
called and only be called as part of the constructor of a derived
class defined as final.

Once the context pool is created, clients can access a
thread-associated context by using ``model_context(optional<int>
context_number)`` and related methods. These methods may be called in
two ways:

- without a context number, the association between thread and context uses the
OpenMP notion of thread number
- with a context number, the method uses the context corresponding to the
provided number

Without a context number, these context access methods are only safe
under the following conditions:

- the caller is the "main thread"
- the caller is an OpenMP team thread *during execution of a parallel region*

With a context number, these context access methods are only safe
under the following conditions:

- no two or more threads simultaneously use the same context number

Methods supporting implicit context parallelism are noted below by
having a reference to this section; as a rule of thumb, any public
method that takes a ``context_number`` argument uses implicit context
parallelism.

Users of this class (derived classes and others) can write their own
parallel operations using implicit contexts, provided they limit
parallel blocks to only use ``const`` methods or methods marked to
support implicit contexts parallelism, and the parallel operations
are:

- without a context number, only with parallelism using OpenMP directives
- with a context number, via a parallelization method that provides a notion of
thread numbers similar in behavior to OpenMP's (i.e. a thread number in
[0, number of threads), not arbitrary values like
``std∷this_thread∷get_id()``)

To determine the greatest implicit context parallelism that can be
achieved in a parallelized operation, ``GetNumberOfThreads(Parallelism
parallelize)`` returns the lesser of the provided ``parallelism`` and
the supported implicit context parallelism.

<h5>Explicit Context Parallelism</h5>

It is also possible to use arbitrary thread models to perform
collision checking in parallel using explicitly created contexts from
this class. Contexts returned from MakeStandaloneModelContext() may be
used in any thread, using only ``const`` methods of this class, or
"explicit context" methods.

Explicit contexts are tracked by this class using ``std∷weak_ptr`` to
track their lifetimes. This mechanism is used by
PerformOperationAgainstAllModelContexts to map an operation over all
collision contexts, whether explicit or implicit.

Methods supporting explicit context parallelism are noted below by
having a reference to this section; as a rule of thumb, any public
method that takes a ``model_context`` first argument uses explicit
context parallelism.

In practice, multi-threaded collision checking with explicit contexts
may look something like the example below.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const Eigen∷VectorXd start_q ...
    const Eigen∷VectorXd sample_q1 ...
    const Eigen∷VectorXd sample_q2 ...
    
    const auto check_edge_to = [&collision_checker, &start_q] (
    const Eigen∷VectorXd& sample_q,
    CollisionCheckerContext* explicit_context) {
    return collision_checker.CheckContextEdgeCollisionFree(
    explicit_context, start_q, sample_q);
    };
    
    const auto context_1 = collision_checker.MakeStandaloneModelContext();
    const auto context_2 = collision_checker.MakeStandaloneModelContext();
    
    auto future_q1 = std∷async(std∷launch∷async, check_edge_to, sample_q1,
    context_1.get());
    auto future_q2 = std∷async(std∷launch∷async, check_edge_to, sample_q2,
    context_2.get());
    
    const double edge_1_valid = future_q1.get();
    const double edge_2_valid = future_q2.get();

.. raw:: html

    </details>

<h5>Mixing Threading Models</h5>

It is possible to support mixed threading models, i.e., using both
OpenMP thread pools and arbitrary threads. In this case, each
arbitrary thread (say, from std∷async) should have its own instance of
a collision checker made using Clone(). Then each arbitrary thread
will have its own implicit context pool.

<h5>Implementing Derived Classes</h5>

Collision checkers deriving from CollisionChecker *must* support
parallel operations from both of the above parallelism models. This is
generally accomplished by placing all mutable state within the
per-thread context. If this cannot be accomplished, the shared mutable
state must be accessed in a thread-safe manner. There are APIs that
depend on SupportsParallelChecking() (e.g.,
CheckConfigsCollisionFree(), CheckEdgeCollisionFreeParallel(),
CheckEdgesCollisionFree(), etc); a derived implementation should
return ``False`` from SupportsParallelChecking() if there is no
meaningful benefit to attempting to do work in parallel (e.g., they
must fully serialize on shared state).)""";
        // Symbol: drake::planning::CollisionChecker::AddCollisionShape
        struct /* AddCollisionShape */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Requests the addition of a shape to a body, both given in
``description``. If added, the shape will belong to the named geometry
group.

Parameter ``group_name``:
    The name of the group to add the geometry to.

Parameter ``description``:
    The data describing the shape and target body.

Returns:
    ``True`` if the shape was added.)""";
        } AddCollisionShape;
        // Symbol: drake::planning::CollisionChecker::AddCollisionShapeToBody
        struct /* AddCollisionShapeToBody */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Requests the addition of ``shape`` to the body A in the checker's
model The added ``shape`` will belong to the named geometry group.

Parameter ``group_name``:
    The name of the group to add the geometry to.

Parameter ``bodyA``:
    The body the shape should be rigidly affixed to.

Parameter ``shape``:
    The requested shape, defined in its canonical frame G.

Parameter ``X_AG``:
    The pose of the shape in body A's frame.

Returns:
    ``True`` if the shape was added.)""";
        } AddCollisionShapeToBody;
        // Symbol: drake::planning::CollisionChecker::AddCollisionShapeToFrame
        struct /* AddCollisionShapeToFrame */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Requests the addition of ``shape`` to the frame A in the checker's
model. The added ``shape`` will belong to the named geometry group.

Parameter ``group_name``:
    The name of the group to add the geometry to.

Parameter ``frameA``:
    The frame the shape should be rigidly affixed to.

Parameter ``shape``:
    The requested shape, defined in its canonical frame G.

Parameter ``X_AG``:
    The pose of the shape in the frame A.

Returns:
    ``True`` if the shape was added.)""";
        } AddCollisionShapeToFrame;
        // Symbol: drake::planning::CollisionChecker::AddCollisionShapes
        struct /* AddCollisionShapes */ {
          // Source: drake/planning/collision_checker.h
          const char* doc_2args =
R"""(Requests the addition of N shapes to N bodies, each given in the set
of ``descriptions``. Each added shape will belong to the named
geometry group.

Parameter ``group_name``:
    The name of the group to add the geometry to.

Parameter ``descriptions``:
    The descriptions of N (shape, body) pairs.

Returns:
    The total number of shapes in ``descriptions`` that got added.)""";
          // Source: drake/planning/collision_checker.h
          const char* doc_1args =
R"""(Requests the addition of a collection of shapes to bodies across
multiple geometry groups. ``geometry_groups`` specifies a collection
of (shape, body) descriptors across multiple geometry groups.

Parameter ``geometry_groups``:
    A map from a named geometry group to the (shape, body) pairs to
    add to that group.

Returns:
    A map from input named geometry group to the *number* of
    geometries added to that group.)""";
        } AddCollisionShapes;
        // Symbol: drake::planning::CollisionChecker::AddedShape
        struct /* AddedShape */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Representation of an "added" shape. These are shapes that get added to
the model via the CollisionChecker's Shape API. They encode the id for
the added geometry and the index of the body (robot or environment) to
which the geometry is affixed.)""";
          // Symbol: drake::planning::CollisionChecker::AddedShape::body_index
          struct /* body_index */ {
            // Source: drake/planning/collision_checker.h
            const char* doc =
R"""(The index of the body the shape was added; could be robot or
environment.)""";
          } body_index;
          // Symbol: drake::planning::CollisionChecker::AddedShape::description
          struct /* description */ {
            // Source: drake/planning/collision_checker.h
            const char* doc =
R"""(The full body description. We have the invariant that ``body_index``
has the body and model instance names recorded in the description.)""";
          } description;
          // Symbol: drake::planning::CollisionChecker::AddedShape::geometry_id
          struct /* geometry_id */ {
            // Source: drake/planning/collision_checker.h
            const char* doc = R"""(The id of the geometry.)""";
          } geometry_id;
        } AddedShape;
        // Symbol: drake::planning::CollisionChecker::AllocateContexts
        struct /* AllocateContexts */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Allocate the per-thread context pool, and discontinue mutable access
to the robot model. This must be called and only be called as part of
the constructor in a derived class defined as final.

Precondition:
    This cannot have already been called for this instance.)""";
        } AllocateContexts;
        // Symbol: drake::planning::CollisionChecker::CalcContextRobotClearance
        struct /* CalcContextRobotClearance */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Explicit Context-based version of CalcRobotClearance().

Raises:
    RuntimeError if ``model_context`` is nullptr.

Raises:
    if ``q`` contains non-finite values.

See also:
    ccb_explicit_contexts "Explicit Context Parallelism".)""";
        } CalcContextRobotClearance;
        // Symbol: drake::planning::CollisionChecker::CalcRobotClearance
        struct /* CalcRobotClearance */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Calculates the distance, ϕ, and distance Jacobian, Jqᵣ_ϕ, for each
potential collision whose distance is less than
``influence_distance``, using the current thread's associated context.

Distances for filtered collisions will not be returned.

Distances between a pair of robot bodies (i.e., where
``collision_types()`` reports ``SelfCollision``) report one body's
index in ``robot_indices()`` and the the other body's in
``other_indices()``; which body appears in which column is arbitrary.

The total number of rows can depend on how the model is defined and
how a particular CollisionChecker instance is implemented (see
MaxNumDistances()).

See also:
    RobotClearance for details on the quantities ϕ and Jqᵣ_ϕ (and
    other details).

Parameter ``context_number``:
    Optional implicit context number.

Raises:
    if ``q`` contains non-finite values.

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".)""";
        } CalcRobotClearance;
        // Symbol: drake::planning::CollisionChecker::CanEvaluateInParallel
        struct /* CanEvaluateInParallel */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    true if this object SupportsParallelChecking() and more than one
    thread is available.)""";
        } CanEvaluateInParallel;
        // Symbol: drake::planning::CollisionChecker::CheckConfigCollisionFree
        struct /* CheckConfigCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Checks a single configuration for collision using the current thread's
associated context.

Parameter ``q``:
    Configuration to check

Parameter ``context_number``:
    Optional implicit context number.

Returns:
    true if collision free, false if in collision.

Raises:
    if ``q`` contains non-finite values.

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".)""";
        } CheckConfigCollisionFree;
        // Symbol: drake::planning::CollisionChecker::CheckConfigsCollisionFree
        struct /* CheckConfigsCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Checks a vector of configurations for collision, evaluating in
parallel when supported and enabled by ``parallelize``.
Parallelization in configuration collision checks is provided using
OpenMP and is supported when both: (1) the collision checker declares
that parallelization is supported (i.e. when
SupportsParallelChecking() is true) and (2) when multiple OpenMP
threads are available for execution. See
collision_checker_parallel_edge "function-level parallelism" for
guidance on proper usage.

Parameter ``configs``:
    Configurations to check

Parameter ``parallelize``:
    How much should collision checks be parallelized?

Returns:
    std∷vector<uint8_t>, one for each configuration in configs. For
    each configuration, 1 if collision free, 0 if in collision.

Raises:
    if ``configs`` contains non-finite values.)""";
        } CheckConfigsCollisionFree;
        // Symbol: drake::planning::CollisionChecker::CheckContextConfigCollisionFree
        struct /* CheckContextConfigCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Explicit Context-based version of CheckConfigCollisionFree().

Raises:
    RuntimeError if model_context is nullptr.

Raises:
    if ``q`` contains non-finite values.

See also:
    ccb_explicit_contexts "Explicit Context Parallelism".)""";
        } CheckContextConfigCollisionFree;
        // Symbol: drake::planning::CollisionChecker::CheckContextEdgeCollisionFree
        struct /* CheckContextEdgeCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Explicit Context-based version of CheckEdgeCollisionFree().

Raises:
    if ``q1`` or ``q2`` contain non-finite values.

Raises:
    RuntimeError if ``model_context`` is nullptr.

See also:
    ccb_explicit_contexts "Explicit Context Parallelism".)""";
        } CheckContextEdgeCollisionFree;
        // Symbol: drake::planning::CollisionChecker::CheckEdgeCollisionFree
        struct /* CheckEdgeCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Checks a single configuration-to-configuration edge for collision,
using the current thread's associated context.

Parameter ``q1``:
    Start configuration for edge.

Parameter ``q2``:
    End configuration for edge.

Parameter ``context_number``:
    Optional implicit context number.

Returns:
    true if collision free, false if in collision.

Raises:
    if ``q1`` or ``q2`` contain non-finite values.

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".)""";
        } CheckEdgeCollisionFree;
        // Symbol: drake::planning::CollisionChecker::CheckEdgeCollisionFreeParallel
        struct /* CheckEdgeCollisionFreeParallel */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Checks a single configuration-to-configuration edge for collision.
Collision check is parallelized via OpenMP when supported. See
collision_checker_parallel_edge "function-level parallelism" for
guidance on proper usage.

Parameter ``q1``:
    Start configuration for edge.

Parameter ``q2``:
    End configuration for edge.

Parameter ``parallelize``:
    How much should edge collision check be parallelized?

Returns:
    true if collision free, false if in collision.

Raises:
    if ``q1`` or ``q2`` contain non-finite values.)""";
        } CheckEdgeCollisionFreeParallel;
        // Symbol: drake::planning::CollisionChecker::CheckEdgesCollisionFree
        struct /* CheckEdgesCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Checks multiple configuration-to-configuration edges for collision.
Collision checks are parallelized via OpenMP when supported and
enabled by ``parallelize``. See collision_checker_parallel_edge
"function-level parallelism" for guidance on proper usage.

Parameter ``edges``:
    Edges to check, each in the form of pair<q1, q2>.

Parameter ``parallelize``:
    How much should edge collision checks be parallelized?

Returns:
    std∷vector<uint8_t>, one for each edge in edges. For each edge, 1
    if collision free, 0 if in collision.

Raises:
    if any vector in ``edges`` contains non-finite values.)""";
        } CheckEdgesCollisionFree;
        // Symbol: drake::planning::CollisionChecker::ClassifyBodyCollisions
        struct /* ClassifyBodyCollisions */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Classifies which robot bodies are in collision (and which type of
collision) for the provided configuration ``q``, using the current
thread's associated context.

Parameter ``context_number``:
    Optional implicit context number.

Returns:
    a vector of collision types arranged in body index order. Only
    entries for robot bodies are guaranteed to be valid; entries for
    environment bodies are populated with kNoCollision, regardless of
    their actual status.

Raises:
    if ``q`` contains non-finite values.

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".)""";
        } ClassifyBodyCollisions;
        // Symbol: drake::planning::CollisionChecker::ClassifyContextBodyCollisions
        struct /* ClassifyContextBodyCollisions */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Explicit Context-based version of ClassifyBodyCollisions().

Raises:
    RuntimeError if ``model_context`` is nullptr.

Raises:
    if ``q`` contains non-finite values.

See also:
    ccb_explicit_contexts "Explicit Context Parallelism".)""";
        } ClassifyContextBodyCollisions;
        // Symbol: drake::planning::CollisionChecker::Clone
        struct /* Clone */ {
          // Source: drake/planning/collision_checker.h
          const char* doc = R"""()""";
        } Clone;
        // Symbol: drake::planning::CollisionChecker::CollisionChecker
        struct /* ctor */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Derived classes declare upon construction whether they support
parallel checking (see SupportsParallelChecking()). If a derived class
does not support parallel checking, it must set
params.implicit_context_parallelism to Parallelism∷None(); otherwise
this constructor will throw.

Raises:
    RuntimeError if params is invalid.

See also:
    CollisionCheckerParams.)""";
          // Source: drake/planning/collision_checker.h
          const char* doc_copy =
R"""(To support Clone(), allow copying (but not move nor assign).)""";
        } ctor;
        // Symbol: drake::planning::CollisionChecker::ComputeConfigurationDistance
        struct /* ComputeConfigurationDistance */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Computes configuration-space distance between the provided
configurations ``q1`` and ``q2``, using the distance function
configured at construction- time or via
SetConfigurationDistanceFunction().

Raises:
    if ``q1`` or ``q2`` contain non-finite values.)""";
        } ComputeConfigurationDistance;
        // Symbol: drake::planning::CollisionChecker::CreatePrototypeContext
        struct /* CreatePrototypeContext */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Collision checkers that use derived context types can override this
implementation to allocate their context type instead.)""";
        } CreatePrototypeContext;
        // Symbol: drake::planning::CollisionChecker::CriticizePaddingMatrix
        struct /* CriticizePaddingMatrix */ {
          // Source: drake/planning/collision_checker.h
          const char* doc = R"""()""";
        } CriticizePaddingMatrix;
        // Symbol: drake::planning::CollisionChecker::DoAddCollisionShapeToBody
        struct /* DoAddCollisionShapeToBody */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Does the work of adding a shape to be rigidly affixed to the body.
Derived checkers can choose to ignore the request, but must return
``nullopt`` if they do so.)""";
        } DoAddCollisionShapeToBody;
        // Symbol: drake::planning::CollisionChecker::DoCalcContextRobotClearance
        struct /* DoCalcContextRobotClearance */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Derived collision checkers are responsible for defining the reported
measurements. But they must adhere to the characteristics documented
on RobotClearance, e.g., one measurement per row. CollisionChecker
guarantees that ``influence_distance`` is finite and non-negative.)""";
        } DoCalcContextRobotClearance;
        // Symbol: drake::planning::CollisionChecker::DoCheckContextConfigCollisionFree
        struct /* DoCheckContextConfigCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Derived collision checkers are responsible for reporting the collision
status of the configuration. CollisionChecker guarantees that the
passed ``model_context`` has been updated with the configuration ``q``
supplied to the public method.)""";
        } DoCheckContextConfigCollisionFree;
        // Symbol: drake::planning::CollisionChecker::DoClassifyContextBodyCollisions
        struct /* DoClassifyContextBodyCollisions */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Derived collision checkers are responsible for choosing a collision
type for each of the robot bodies. They should adhere to the semantics
documented for ClassifyBodyCollisions. CollisionChecker guarantees
that the passed ``model_context`` has been updated with the
configuration ``q`` supplied to the public method.)""";
        } DoClassifyContextBodyCollisions;
        // Symbol: drake::planning::CollisionChecker::DoClone
        struct /* DoClone */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Derived collision checkers implement can make use of the protected
copy constructor to implement DoClone().)""";
        } DoClone;
        // Symbol: drake::planning::CollisionChecker::DoMaxContextNumDistances
        struct /* DoMaxContextNumDistances */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Derived collision checkers must implement the semantics documented for
MaxNumDistances. CollisionChecker does nothing; it just calls this
method.)""";
        } DoMaxContextNumDistances;
        // Symbol: drake::planning::CollisionChecker::DoUpdateContextPositions
        struct /* DoUpdateContextPositions */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Derived collision checkers can do further work in this function in
response to updates to the MultibodyPlant positions. CollisionChecker
guarantees that ``model_context`` will not be nullptr and that the new
positions are present in model_context->plant_context().)""";
        } DoUpdateContextPositions;
        // Symbol: drake::planning::CollisionChecker::GetAllAddedCollisionShapes
        struct /* GetAllAddedCollisionShapes */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Gets all checker geometries currently added across the whole checker.

Returns:
    A mapping from each geometry group name to the collection of
    (shape, body) descriptions in that group.)""";
        } GetAllAddedCollisionShapes;
        // Symbol: drake::planning::CollisionChecker::GetFilteredCollisionMatrix
        struct /* GetFilteredCollisionMatrix */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Gets the "active" collision filter matrix.)""";
        } GetFilteredCollisionMatrix;
        // Symbol: drake::planning::CollisionChecker::GetLargestPadding
        struct /* GetLargestPadding */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Gets the current largest collision padding across all (robot, *) body
pairs. This excludes the meaningless zeros on the diagonal and
environment-environment pairs; the return value *can* be negative.)""";
        } GetLargestPadding;
        // Symbol: drake::planning::CollisionChecker::GetMutableSetupModel
        struct /* GetMutableSetupModel */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    a mutable reference to the robot model.

Raises:
    RuntimeError if IsInitialSetup() == false.)""";
        } GetMutableSetupModel;
        // Symbol: drake::planning::CollisionChecker::GetNominalFilteredCollisionMatrix
        struct /* GetNominalFilteredCollisionMatrix */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns the "nominal" collision filter matrix. The nominal matrix is
initialized at construction time and represents the configuration of
the model's plant and scene graph. It serves as a reference point to
assess any changes to collision filters beyond this checker's
intrinsic model.

Collisions between bodies A and B are filtered in the following cases:

- There exists a welded path between A and B.
- SceneGraph has filtered the collisions between *all* pairs of geometries
of A and B.

Note: SceneGraph allows arbitrary collision filter configuration at
the geometry* level. The filters on one geometry of body need not be
the same as another geometry on the same body. CollisionChecker is
body centric. It requires all geometries on a body to be filtered
homogeneously. A SceneGraph that violates this stricter requirement
cannot be used in a CollisionChecker. It is highly unlikely that a
SceneGraph instance will ever be in this configuration by accident.)""";
        } GetNominalFilteredCollisionMatrix;
        // Symbol: drake::planning::CollisionChecker::GetPaddingBetween
        struct /* GetPaddingBetween */ {
          // Source: drake/planning/collision_checker.h
          const char* doc_2args_bodyA_index_bodyB_index =
R"""(Gets the padding value for the pair of bodies specified. If the body
indices are the same, zero will always be returned.

Raises:
    RuntimeError if either body index is out of range.)""";
          // Source: drake/planning/collision_checker.h
          const char* doc_2args_bodyA_bodyB = R"""(Overload that uses body references.)""";
        } GetPaddingBetween;
        // Symbol: drake::planning::CollisionChecker::GetPaddingMatrix
        struct /* GetPaddingMatrix */ {
          // Source: drake/planning/collision_checker.h
          const char* doc = R"""(Gets the collision padding matrix.)""";
        } GetPaddingMatrix;
        // Symbol: drake::planning::CollisionChecker::GetZeroConfiguration
        struct /* GetZeroConfiguration */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    a generalized position vector, sized according to the full model,
    whose values are all zero.

Warning:
    A zero vector is not necessarily a valid configuration, e.g., in
    case the configuration has quaternions, or position constraints,
    or etc.)""";
        } GetZeroConfiguration;
        // Symbol: drake::planning::CollisionChecker::InterpolateBetweenConfigurations
        struct /* InterpolateBetweenConfigurations */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Interpolates between provided configurations ``q1`` and ``q2``.

Parameter ``ratio``:
    Interpolation ratio.

Returns:
    Interpolated configuration.

Raises:
    RuntimeError if ratio is not in range [0, 1].

Raises:
    if ``q1`` or ``q2`` contain non-finite values.

See also:
    ConfigurationInterpolationFunction for more.)""";
        } InterpolateBetweenConfigurations;
        // Symbol: drake::planning::CollisionChecker::IsCollisionFilteredBetween
        struct /* IsCollisionFilteredBetween */ {
          // Source: drake/planning/collision_checker.h
          const char* doc_2args_bodyA_index_bodyB_index =
R"""(Checks if collision is filtered between the two bodies specified.
Note: collision between two environment bodies is *always* filtered.

Raises:
    RuntimeError if either body index is out of range.)""";
          // Source: drake/planning/collision_checker.h
          const char* doc_2args_bodyA_bodyB = R"""(Overload that uses body references.)""";
        } IsCollisionFilteredBetween;
        // Symbol: drake::planning::CollisionChecker::IsInitialSetup
        struct /* IsInitialSetup */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    true if called during initial setup (before AllocateContexts() is
    called).)""";
        } IsInitialSetup;
        // Symbol: drake::planning::CollisionChecker::IsPartOfRobot
        struct /* IsPartOfRobot */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    true if the indicated body is part of the robot.)""";
        } IsPartOfRobot;
        // Symbol: drake::planning::CollisionChecker::MakeStandaloneConfigurationDistanceFunction
        struct /* MakeStandaloneConfigurationDistanceFunction */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    a functor that captures this object, so it can be used like a free
    function. The returned functor is only valid during the lifetime
    of this object. The math of the function is equivalent to
    ComputeConfigurationDistance().

Warning:
    do not pass this standalone function back into
    SetConfigurationDistanceFunction() function; doing so would create
    an infinite loop.

Raises:
    if ``q1`` or ``q2`` contain non-finite values.)""";
        } MakeStandaloneConfigurationDistanceFunction;
        // Symbol: drake::planning::CollisionChecker::MakeStandaloneConfigurationInterpolationFunction
        struct /* MakeStandaloneConfigurationInterpolationFunction */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    a functor that captures this object, so it can be used like a free
    function. The returned functor is only valid during the lifetime
    of this object. The math of the function is equivalent to
    InterpolateBetweenConfigurations().

Warning:
    do not pass this standalone function back into our
    SetConfigurationInterpolationFunction() function; doing so would
    create an infinite loop.)""";
        } MakeStandaloneConfigurationInterpolationFunction;
        // Symbol: drake::planning::CollisionChecker::MakeStandaloneModelContext
        struct /* MakeStandaloneModelContext */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Make and track a CollisionCheckerContext. The returned context will
participate in PerformOperationAgainstAllModelContexts() until it is
destroyed.)""";
        } MakeStandaloneModelContext;
        // Symbol: drake::planning::CollisionChecker::MaxContextNumDistances
        struct /* MaxContextNumDistances */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Explicit Context-based version of MaxNumDistances().

See also:
    ccb_explicit_contexts "Explicit Context Parallelism".)""";
        } MaxContextNumDistances;
        // Symbol: drake::planning::CollisionChecker::MaxNumDistances
        struct /* MaxNumDistances */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns an upper bound on the number of distances returned by
CalcRobotClearance(), using the current thread's associated context.

Parameter ``context_number``:
    Optional implicit context number.

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".)""";
        } MaxNumDistances;
        // Symbol: drake::planning::CollisionChecker::MaybeGetUniformRobotEnvironmentPadding
        struct /* MaybeGetUniformRobotEnvironmentPadding */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(If the padding between all robot bodies and environment bodies is the
same, returns the common padding value. Returns nullopt otherwise.)""";
        } MaybeGetUniformRobotEnvironmentPadding;
        // Symbol: drake::planning::CollisionChecker::MaybeGetUniformRobotRobotPadding
        struct /* MaybeGetUniformRobotRobotPadding */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(If the padding between all pairs of robot bodies is the same, returns
the common padding value. Returns nullopt otherwise.)""";
        } MaybeGetUniformRobotRobotPadding;
        // Symbol: drake::planning::CollisionChecker::MeasureContextEdgeCollisionFree
        struct /* MeasureContextEdgeCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Explicit Context-based version of MeasureEdgeCollisionFree().

Raises:
    RuntimeError if ``model_context`` is nullptr.

Raises:
    if ``q1`` or ``q2`` contain non-finite values.

See also:
    ccb_explicit_contexts "Explicit Context Parallelism".)""";
        } MeasureContextEdgeCollisionFree;
        // Symbol: drake::planning::CollisionChecker::MeasureEdgeCollisionFree
        struct /* MeasureEdgeCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Checks a single configuration-to-configuration edge for collision,
using the current thread's associated context.

Parameter ``q1``:
    Start configuration for edge.

Parameter ``q2``:
    End configuration for edge.

Parameter ``context_number``:
    Optional implicit context number.

Returns:
    A measure of how much of the edge is collision free.

Raises:
    if ``q1`` or ``q2`` contain non-finite values.

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".)""";
        } MeasureEdgeCollisionFree;
        // Symbol: drake::planning::CollisionChecker::MeasureEdgeCollisionFreeParallel
        struct /* MeasureEdgeCollisionFreeParallel */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Checks a single configuration-to-configuration edge for collision.
Collision check is parallelized via OpenMP when supported. See
collision_checker_parallel_edge "function-level parallelism" for
guidance on proper usage.

Parameter ``q1``:
    Start configuration for edge.

Parameter ``q2``:
    End configuration for edge.

Parameter ``parallelize``:
    How much should edge collision check be parallelized?

Returns:
    A measure of how much of the edge is collision free.

Raises:
    if ``q1`` or ``q2`` contain non-finite values.)""";
        } MeasureEdgeCollisionFreeParallel;
        // Symbol: drake::planning::CollisionChecker::MeasureEdgesCollisionFree
        struct /* MeasureEdgesCollisionFree */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Checks multiple configuration-to-configuration edge for collision.
Collision checks are parallelized via OpenMP when supported and
enabled by ``parallelize``. See collision_checker_parallel_edge
"function-level parallelism" for guidance on proper usage.

Parameter ``edges``:
    Edges to check, each in the form of pair<q1, q2>.

Parameter ``parallelize``:
    How much should edge collision checks be parallelized?

Returns:
    A measure of how much of each edge is collision free. The iᵗʰ
    entry is the result for the iᵗʰ edge.

Raises:
    if any vector in ``edges`` contains non-finite values.)""";
        } MeasureEdgesCollisionFree;
        // Symbol: drake::planning::CollisionChecker::PerformOperationAgainstAllModelContexts
        struct /* PerformOperationAgainstAllModelContexts */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Allows externally-provided operations that must be performed against
all contexts in the per-thread context pool, and any standalone
contexts made with MakeStandaloneModelContext().

For any standalone contexts, note that it is illegal to mutate a
context from two different threads. No other threads should be
mutating any of our standalone contexts when this function is called.)""";
        } PerformOperationAgainstAllModelContexts;
        // Symbol: drake::planning::CollisionChecker::RemoveAddedGeometries
        struct /* RemoveAddedGeometries */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Removes all of the given added shapes (if they exist) from the
checker.)""";
        } RemoveAddedGeometries;
        // Symbol: drake::planning::CollisionChecker::RemoveAllAddedCollisionShapes
        struct /* RemoveAllAddedCollisionShapes */ {
          // Source: drake/planning/collision_checker.h
          const char* doc_1args =
R"""(Removes all added checker geometries which belong to the named group.)""";
          // Source: drake/planning/collision_checker.h
          const char* doc_0args =
R"""(Removes all added checker geometries from all geometry groups.)""";
        } RemoveAllAddedCollisionShapes;
        // Symbol: drake::planning::CollisionChecker::SetCollisionFilterMatrix
        struct /* SetCollisionFilterMatrix */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Sets the "active" collision filter matrix

Parameter ``filter_matrix``:
    must meet the above conditions to be a "consistent" collision
    filter matrix.

Raises:
    RuntimeError if the given matrix is incompatible with this
    collision checker, or if it is inconsistent.)""";
        } SetCollisionFilterMatrix;
        // Symbol: drake::planning::CollisionChecker::SetCollisionFilteredBetween
        struct /* SetCollisionFilteredBetween */ {
          // Source: drake/planning/collision_checker.h
          const char* doc_3args_bodyA_index_bodyB_index_filter_collision =
R"""(Declares the body pair (bodyA, bodyB) to be filtered (or not) based on
``filter_collision``.

Parameter ``filter_collision``:
    Sets the to body pair to be filtered if ``True``.

Raises:
    RuntimeError if either body index is out of range.

Raises:
    RuntimeError if both indices refer to the same body.

Raises:
    RuntimeError if both indices refer to environment bodies.)""";
          // Source: drake/planning/collision_checker.h
          const char* doc_3args_bodyA_bodyB_filter_collision = R"""(Overload that uses body references.)""";
        } SetCollisionFilteredBetween;
        // Symbol: drake::planning::CollisionChecker::SetCollisionFilteredWithAllBodies
        struct /* SetCollisionFilteredWithAllBodies */ {
          // Source: drake/planning/collision_checker.h
          const char* doc_1args_body_index =
R"""(Declares that body pair (B, O) is filtered (for all bodies O in this
checker's plant).

Raises:
    RuntimeError if ``body_index`` is out of range.

Raises:
    RuntimeError if ``body_index`` refers to an environment body.)""";
          // Source: drake/planning/collision_checker.h
          const char* doc_1args_body = R"""(Overload that uses body references.)""";
        } SetCollisionFilteredWithAllBodies;
        // Symbol: drake::planning::CollisionChecker::SetConfigurationDistanceFunction
        struct /* SetConfigurationDistanceFunction */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Sets the configuration distance function to ``distance_function``.

Precondition:
    distance_function satisfies the requirements documented on
    ConfigurationDistanceFunction and a
    DistanceAndInterpolationProvider is not already in use.

Precondition:
    the collision checker was created with separate distance and
    interpolation functions, not a combined
    DistanceAndInterpolationProvider.

Note:
    the ``distance_function`` object will be copied and retained by
    this collision checker, so if the function has any lambda-captured
    data then that data must outlive this collision checker.)""";
        } SetConfigurationDistanceFunction;
        // Symbol: drake::planning::CollisionChecker::SetConfigurationInterpolationFunction
        struct /* SetConfigurationInterpolationFunction */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Sets the configuration interpolation function to
``interpolation_function``.

Parameter ``interpolation_function``:
    a functor, or nullptr. If nullptr, the default function will be
    configured and used.

Precondition:
    interpolation_function satisfies the requirements documented on
    ConfigurationInterpolationFunction, or is nullptr and a
    DistanceAndInterpolationProvider is not already in use.

Precondition:
    the collision checker was created with separate distance and
    interpolation functions, not a combined
    DistanceAndInterpolationProvider.

Note:
    the ``interpolation_function`` object will be copied and retained
    by this collision checker, so if the function has any
    lambda-captured data then that data must outlive this collision
    checker.

Note:
    the default function uses linear interpolation for most variables,
    and uses slerp for quaternion valued variables.)""";
        } SetConfigurationInterpolationFunction;
        // Symbol: drake::planning::CollisionChecker::SetDistanceAndInterpolationProvider
        struct /* SetDistanceAndInterpolationProvider */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Sets the distance and interpolation provider to use. Note that in case
any of the (to-be-deprecated) separate distance and interpolation
functions were in use, this supplants *both* of them.

Precondition:
    provider satisfies the requirements documents on
    DistanceAndInterpolationProvider.)""";
        } SetDistanceAndInterpolationProvider;
        // Symbol: drake::planning::CollisionChecker::SetPaddingAllRobotEnvironmentPairs
        struct /* SetPaddingAllRobotEnvironmentPairs */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Sets the padding for all (robot, environment) pairs.

Raises:
    RuntimeError if the collision_checker_padding_prereqs
    "configuration prerequisites" are not met.)""";
        } SetPaddingAllRobotEnvironmentPairs;
        // Symbol: drake::planning::CollisionChecker::SetPaddingAllRobotRobotPairs
        struct /* SetPaddingAllRobotRobotPairs */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Sets the padding for all (robot, robot) pairs.

Raises:
    RuntimeError if the collision_checker_padding_prereqs
    "configuration prerequisites" are not met.)""";
        } SetPaddingAllRobotRobotPairs;
        // Symbol: drake::planning::CollisionChecker::SetPaddingBetween
        struct /* SetPaddingBetween */ {
          // Source: drake/planning/collision_checker.h
          const char* doc_3args_bodyA_index_bodyB_index_padding =
R"""(Sets the padding value for the pair of bodies specified.

Raises:
    RuntimeError if the collision_checker_padding_prereqs
    "configuration prerequisites" are not met or ``bodyA_index ==
    bodyB_index``.)""";
          // Source: drake/planning/collision_checker.h
          const char* doc_3args_bodyA_bodyB_padding = R"""(Overload that uses body references.)""";
        } SetPaddingBetween;
        // Symbol: drake::planning::CollisionChecker::SetPaddingMatrix
        struct /* SetPaddingMatrix */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Sets the collision padding matrix. Note that this matrix contains all
padding data, both robot-robot "self" padding, and robot-environment
padding. ``collision_padding`` must have the following properties to
be considered valid.

- It is a square NxN matrix (where N is the total number of bodies).
- Diagonal values are all zero.
- Entries involving only environment bodies are all zero.
- It is symmetric.
- All values are finite.

Raises:
    RuntimeError if ``collision_padding`` doesn't have the enumerated
    properties.)""";
        } SetPaddingMatrix;
        // Symbol: drake::planning::CollisionChecker::SetPaddingOneRobotBodyAllEnvironmentPairs
        struct /* SetPaddingOneRobotBodyAllEnvironmentPairs */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Sets the environment collision padding for the provided robot body
with respect to all environment bodies.

Raises:
    RuntimeError if the collision_checker_padding_prereqs
    "configuration prerequisites" are not met.)""";
        } SetPaddingOneRobotBodyAllEnvironmentPairs;
        // Symbol: drake::planning::CollisionChecker::SupportsParallelChecking
        struct /* SupportsParallelChecking */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Does the collision checker support true parallel collision checks?

Returns:
    true if parallel checking is supported.)""";
        } SupportsParallelChecking;
        // Symbol: drake::planning::CollisionChecker::UpdateCollisionFilters
        struct /* UpdateCollisionFilters */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Derived collision checkers can do further work in this function in
response to changes in collision filters. This is called after any
changes are made to the collision filter matrix.)""";
        } UpdateCollisionFilters;
        // Symbol: drake::planning::CollisionChecker::UpdateContextPositions
        struct /* UpdateContextPositions */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Explicit Context-based version of UpdatePositions().

Raises:
    RuntimeError if ``model_context`` is ``nullptr``.

Raises:
    if ``q`` contains non-finite values.

See also:
    ccb_explicit_contexts "Explicit Context Parallelism".)""";
        } UpdateContextPositions;
        // Symbol: drake::planning::CollisionChecker::UpdatePositions
        struct /* UpdatePositions */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Updates the generalized positions ``q`` in the implicit context
specified and returns a reference to the MultibodyPlant's now-updated
context. The implicit context is either that specified by
``context_number``, or when nullopt the context to be used with the
current OpenMP thread.

Parameter ``context_number``:
    Optional implicit context number.

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".

Raises:
    if ``q`` contains non-finite values.)""";
        } UpdatePositions;
        // Symbol: drake::planning::CollisionChecker::distance_and_interpolation_provider
        struct /* distance_and_interpolation_provider */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Gets the DistanceAndInterpolationProvider in use.)""";
        } distance_and_interpolation_provider;
        // Symbol: drake::planning::CollisionChecker::edge_step_size
        struct /* edge_step_size */ {
          // Source: drake/planning/collision_checker.h
          const char* doc = R"""(Gets the current edge step size.)""";
        } edge_step_size;
        // Symbol: drake::planning::CollisionChecker::get_body
        struct /* get_body */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    a const body reference to a body in the full model's plant for the
    given ``body_index``.)""";
        } get_body;
        // Symbol: drake::planning::CollisionChecker::model
        struct /* model */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    a const reference to the full model.)""";
        } model;
        // Symbol: drake::planning::CollisionChecker::model_context
        struct /* model_context */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Accesses a collision checking context from within the implicit context
pool owned by this collision checker.

Parameter ``context_number``:
    Optional implicit context number.

Returns:
    a const reference to either the collision checking context given
    by the ``context_number``, or when nullopt the context to be used
    with the current OpenMP thread.

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".)""";
        } model_context;
        // Symbol: drake::planning::CollisionChecker::num_allocated_contexts
        struct /* num_allocated_contexts */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    the number of internal (not standalone) per-thread contexts.)""";
        } num_allocated_contexts;
        // Symbol: drake::planning::CollisionChecker::plant
        struct /* plant */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Returns:
    a const reference to the full model's plant.)""";
        } plant;
        // Symbol: drake::planning::CollisionChecker::plant_context
        struct /* plant_context */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Accesses a multibody plant sub-context context from within the
implicit context pool owned by this collision checker.

Parameter ``context_number``:
    Optional implicit context number.

Returns:
    a const reference to the multibody plant sub-context within the
    context given by the ``context_number``, or when nullopt the
    context to be used with the current OpenMP thread.

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".)""";
        } plant_context;
        // Symbol: drake::planning::CollisionChecker::robot_model_instances
        struct /* robot_model_instances */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Gets the set of model instances belonging to the robot. The returned
vector has no duplicates and is in sorted order.)""";
        } robot_model_instances;
        // Symbol: drake::planning::CollisionChecker::set_edge_step_size
        struct /* set_edge_step_size */ {
          // Source: drake/planning/collision_checker.h
          const char* doc =
R"""(Sets the edge step size to ``edge_step_size``.

Raises:
    RuntimeError if ``edge_step_size`` is not positive.)""";
        } set_edge_step_size;
      } CollisionChecker;
      // Symbol: drake::planning::CollisionCheckerContext
      struct /* CollisionCheckerContext */ {
        // Source: drake/planning/collision_checker_context.h
        const char* doc =
R"""(This class represents the data necessary for CollisionChecker to
operate safely across multiple threads in its ``const`` API. Instances
of this class are owned and managed by a particular CollisionChecker.

If using OMP to perform parallel const queries on a CollisionChecker,
it will never be necessary to interact with CollisionCheckerContext
instances. Only if using some other threading paradigm will it be
necessary to work with "stand alone" instances. See CollisionChecker's
documentation for more details.

In all cases, modifying context should happen through
CollisionChecker∷PerformOperationAgainstAllModelContexts(). Modifying
the contained Drake Contexts directly is generally erroneous.)""";
        // Symbol: drake::planning::CollisionCheckerContext::Clone
        struct /* Clone */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc = R"""()""";
        } Clone;
        // Symbol: drake::planning::CollisionCheckerContext::CollisionCheckerContext
        struct /* ctor */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc =
R"""(The resulting object stores an alias to ``model``; the passed model
should have a lifetime greater than the constructed object.

Precondition:
    model is not null.)""";
          // Source: drake/planning/collision_checker_context.h
          const char* doc_copy =
R"""(Derived classes can use this copy constructor to help implement their
own DoClone() methods.)""";
        } ctor;
        // Symbol: drake::planning::CollisionCheckerContext::DoClone
        struct /* DoClone */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc =
R"""(Allow derived context types to implement additional clone behavior.)""";
        } DoClone;
        // Symbol: drake::planning::CollisionCheckerContext::GetQueryObject
        struct /* GetQueryObject */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc =
R"""(Gets the scene graph geometry query object.)""";
        } GetQueryObject;
        // Symbol: drake::planning::CollisionCheckerContext::model_context
        struct /* model_context */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc = R"""(Gets the contained model context.)""";
        } model_context;
        // Symbol: drake::planning::CollisionCheckerContext::mutable_model_context
        struct /* mutable_model_context */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc = R"""()""";
        } mutable_model_context;
        // Symbol: drake::planning::CollisionCheckerContext::mutable_plant_context
        struct /* mutable_plant_context */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc = R"""()""";
        } mutable_plant_context;
        // Symbol: drake::planning::CollisionCheckerContext::mutable_scene_graph_context
        struct /* mutable_scene_graph_context */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc = R"""()""";
        } mutable_scene_graph_context;
        // Symbol: drake::planning::CollisionCheckerContext::plant_context
        struct /* plant_context */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc = R"""(Gets the contained plant context.)""";
        } plant_context;
        // Symbol: drake::planning::CollisionCheckerContext::scene_graph_context
        struct /* scene_graph_context */ {
          // Source: drake/planning/collision_checker_context.h
          const char* doc = R"""(Gets the contained scene graph context.)""";
        } scene_graph_context;
      } CollisionCheckerContext;
      // Symbol: drake::planning::CollisionCheckerParams
      struct /* CollisionCheckerParams */ {
        // Source: drake/planning/collision_checker_params.h
        const char* doc =
R"""(A set of common constructor parameters for a CollisionChecker. Not all
subclasses of CollisionChecker will necessarily support this
configuration struct, but many do so.)""";
        // Symbol: drake::planning::CollisionCheckerParams::configuration_distance_function
        struct /* configuration_distance_function */ {
          // Source: drake/planning/collision_checker_params.h
          const char* doc =
R"""(Configuration (probably weighted) distance function.

Note:
    Either a DistanceAndInterpolationProvider OR a
    ConfigurationDistanceFunction may be provided, not both. If
    neither is provided, a LinearDistanceAndInterpolationProvider with
    default weights is used.

Note:
    the ``configuration_distance_function`` object will be copied and
    retained by a collision checker, so if the function has any
    lambda-captured data then that data must outlive the collision
    checker.)""";
        } configuration_distance_function;
        // Symbol: drake::planning::CollisionCheckerParams::distance_and_interpolation_provider
        struct /* distance_and_interpolation_provider */ {
          // Source: drake/planning/collision_checker_params.h
          const char* doc =
R"""(A DistanceAndInterpolationProvider to support configuration distance
and interpolation operations.

Note:
    Either a DistanceAndInterpolationProvider OR a
    ConfigurationDistanceFunction may be provided, not both. If
    neither is provided, a LinearDistanceAndInterpolationProvider with
    default weights is used.)""";
        } distance_and_interpolation_provider;
        // Symbol: drake::planning::CollisionCheckerParams::edge_step_size
        struct /* edge_step_size */ {
          // Source: drake/planning/collision_checker_params.h
          const char* doc =
R"""(Step size for edge checking; in units compatible with the
configuration distance function. Collision checking of edges q1->q2 is
performed by interpolating from q1 to q2 at edge_step_size steps and
checking the interpolated configuration for collision. The value must
be positive.)""";
        } edge_step_size;
        // Symbol: drake::planning::CollisionCheckerParams::env_collision_padding
        struct /* env_collision_padding */ {
          // Source: drake/planning/collision_checker_params.h
          const char* doc =
R"""(Additional padding to apply to all robot-environment collision
queries. If distance between robot and environment is less than
padding, the checker reports a collision.)""";
        } env_collision_padding;
        // Symbol: drake::planning::CollisionCheckerParams::implicit_context_parallelism
        struct /* implicit_context_parallelism */ {
          // Source: drake/planning/collision_checker_params.h
          const char* doc =
R"""(Specify how many contexts should be allocated to support collision
checker implicit context parallelism. Defaults to the maximum
parallelism. If the specific collision checker type in use declares
that it *does not* support parallel queries, then implicit context
parallelism is set to None().

See also:
    ccb_implicit_contexts "Implicit Context Parallelism".)""";
        } implicit_context_parallelism;
        // Symbol: drake::planning::CollisionCheckerParams::model
        struct /* model */ {
          // Source: drake/planning/collision_checker_params.h
          const char* doc =
R"""(A RobotDiagram model of the robot and environment. Must not be
nullptr.)""";
        } model;
        // Symbol: drake::planning::CollisionCheckerParams::robot_model_instances
        struct /* robot_model_instances */ {
          // Source: drake/planning/collision_checker_params.h
          const char* doc =
R"""(A vector of model instance indices that identify which model instances
belong to the robot. The list must be non-empty and must not include
the world model instance.)""";
        } robot_model_instances;
        // Symbol: drake::planning::CollisionCheckerParams::self_collision_padding
        struct /* self_collision_padding */ {
          // Source: drake/planning/collision_checker_params.h
          const char* doc =
R"""(Additional padding to apply to all robot-robot self collision queries.
If distance between robot and itself is less than padding, the checker
reports a collision.)""";
        } self_collision_padding;
      } CollisionCheckerParams;
      // Symbol: drake::planning::ConfigurationDistanceFunction
      struct /* ConfigurationDistanceFunction */ {
        // Source: drake/planning/collision_checker_params.h
        const char* doc =
R"""(Configuration distance takes two configurations of the robot, q1 and
q2, both as Eigen∷VectorXd, and returns (potentially weighted) C-space
distance as a double. The returned distance will be strictly
non-negative.

To be valid, the function must satisfy the following condition:

- dist(q, q) ≡ 0

for values of q that are valid for the CollisionChecker's plant.)""";
      } ConfigurationDistanceFunction;
      // Symbol: drake::planning::ConfigurationInterpolationFunction
      struct /* ConfigurationInterpolationFunction */ {
        // Source: drake/planning/collision_checker_params.h
        const char* doc =
R"""(Configuration interpolation function takes two configurations of the
robot, q1, and q2, both as Eigen∷VectorXd, plus a ratio, r, in [0, 1]
and returns the interpolated configuration. Behavior of the function
for values of r outside of the range [0,1] is undefined.

To be valid, the function must satisfy the following conditions:

- interpolate(q1, q2, 0) ≡ q1
- interpolate(q1, q2, 1) ≡ q2
- interpolate(q, q, r) ≡ q, for all r in [0, 1]

for values of q, q1, and q2 that are valid for the CollisionChecker's
plant.)""";
      } ConfigurationInterpolationFunction;
      // Symbol: drake::planning::DistanceAndInterpolationProvider
      struct /* DistanceAndInterpolationProvider */ {
        // Source: drake/planning/distance_and_interpolation_provider.h
        const char* doc =
R"""(This class represents the base interface for performing configuration
distance and interpolation operations, used by CollisionChecker. See
LinearDistanceAndInterpolationProvider for an implementation covering
common "linear" distance and interpolation behavior.

Configuration distance and interpolation are necessary for a
CollisionChecker to perform edge collision checks, and an essential
part of many motion planning problems. The C-spaces for many planning
problems combine joints with widely differing effects (e.g. for a
given angular change, the shoulder joint of a robot arm results in
much more significant motion than the same change on a finger joint)
or units (e.g. a mobile robot with translation in meters and yaw in
radians). As a result, it is often necessary to weight elements of the
configuration differently when computing configuration distance.

Likewise, in more complex C-spaces, it may be necessary to perform
more complex interpolation behavior (e.g. when planning for a mobile
robot whose motion is modelled via Dubbins or Reeds-Shepp paths).

Configuration distance takes two configurations of the robot, from and
to, both as Eigen∷VectorXd, and returns (potentially weighted) C-space
distance as a double. The returned distance will be strictly
non-negative.

To be valid, distance must satisfy the following condition:

- ComputeConfigurationDistance(q, q) ≡ 0

for values of q that are valid for the C-space in use.

Configuration interpolation takes two configurations of the robot,
from and to, both as Eigen∷VectorXd, plus a ratio in [0, 1] and
returns the interpolated configuration.

To be valid, interpolation must satisfy the following conditions:

- InterpolateBetweenConfigurations(from, to, 0) ≡ from
- InterpolateBetweenConfigurations(from, to, 1) ≡ to
- InterpolateBetweenConfigurations(q, q, ratio) ≡ q, for all ratio in [0, 1]

for values of q, from, and to that are valid for the C-space in use.)""";
        // Symbol: drake::planning::DistanceAndInterpolationProvider::ComputeConfigurationDistance
        struct /* ComputeConfigurationDistance */ {
          // Source: drake/planning/distance_and_interpolation_provider.h
          const char* doc =
R"""(Computes the configuration distance from the provided configuration
``from`` to the provided configuration ``to``. The returned distance
will be strictly non-negative.

Precondition:
    from.size() == to.size().

Raises:
    if ``from`` or ``to`` contain non-finite values.)""";
        } ComputeConfigurationDistance;
        // Symbol: drake::planning::DistanceAndInterpolationProvider::DistanceAndInterpolationProvider
        struct /* ctor */ {
          // Source: drake/planning/distance_and_interpolation_provider.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::planning::DistanceAndInterpolationProvider::DoComputeConfigurationDistance
        struct /* DoComputeConfigurationDistance */ {
          // Source: drake/planning/distance_and_interpolation_provider.h
          const char* doc =
R"""(Derived distance and interpolation providers must implement distance
computation. The returned distance must be non-negative.
DistanceAndInterpolationProvider ensures that ``from`` and ``to`` are
the same size.

Base class guarantees that ``from`` and ``to`` contain only finite
values.)""";
        } DoComputeConfigurationDistance;
        // Symbol: drake::planning::DistanceAndInterpolationProvider::DoInterpolateBetweenConfigurations
        struct /* DoInterpolateBetweenConfigurations */ {
          // Source: drake/planning/distance_and_interpolation_provider.h
          const char* doc =
R"""(Derived distance and interpolation providers must implement
interpolation. The returned configuration must be the same size as
``from`` and ``to``. DistanceAndInterpolationProvider ensures that
``from`` and ``to`` are the same size and that ``ratio`` is in [0, 1].

Base class guarantees that ``from`` and ``to`` contain only finite
values.)""";
        } DoInterpolateBetweenConfigurations;
        // Symbol: drake::planning::DistanceAndInterpolationProvider::InterpolateBetweenConfigurations
        struct /* InterpolateBetweenConfigurations */ {
          // Source: drake/planning/distance_and_interpolation_provider.h
          const char* doc =
R"""(Returns the interpolated configuration between ``from`` and ``to`` at
``ratio``.

Precondition:
    from.size() == to.size().

Precondition:
    ratio in [0, 1].

Raises:
    if ``from`` or ``to`` contain non-finite values.)""";
        } InterpolateBetweenConfigurations;
      } DistanceAndInterpolationProvider;
      // Symbol: drake::planning::DofMask
      struct /* DofMask */ {
        // Source: drake/planning/dof_mask.h
        const char* doc =
R"""(A mask on the degrees of freedom (dofs) of a MultibodyPlant instance,
partitioning the plant's dofs into "selected" and "unselected" dofs.

A DofMask has two measures: its "size" and its "count". The size is
the number of dofs it can mask in total (the selected and unselected
dofs together). It must be equal to the number of dofs contained in a
MultibodyPlant instance (its ``q`s). The interpretation of a DofMask's
indices come from the plant's ordering of its own dofs and their
indices. A DofMask instance's valid index values lie in the range `[0,
size())``. The *count* of the DofMask is the number of *selected*
dofs.

Currently, DofMask is only compatible with MultibodyPlant instances
where the generalized velocities *are* the same as the time
derivatives of the generalized positions. Or, in other words, ``vᵢ =
q̇ᵢ``. This property holds for all single-dof joints but is not
generally true for all joints (e.g., floating joints or roll-pitch-yaw
ball joints).)""";
        // Symbol: drake::planning::DofMask::Complement
        struct /* Complement */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(Returns a new mask such that its selected dofs are ``this`` mask's
unselected dofs and vice versa.)""";
        } Complement;
        // Symbol: drake::planning::DofMask::DofMask
        struct /* ctor */ {
          // Source: drake/planning/dof_mask.h
          const char* doc_default =
R"""(Default constructor; creates a mask with no dofs (size() = count() = 0
).)""";
          // Source: drake/planning/dof_mask.h
          const char* doc_by_size =
R"""(Full/empty constructor.

This allows for construction of a mask with the given ``size`` where
all dofs are either selected (``value = true``) or unselected.

Precondition:
    size >= 0)""";
          // Source: drake/planning/dof_mask.h
          const char* doc_init_list =
R"""(Constructs a DofMask from an initializer list of bool.)""";
          // Source: drake/planning/dof_mask.h
          const char* doc_vector_bool =
R"""(Constructs a DofMask from a vector of bool.)""";
        } ctor;
        // Symbol: drake::planning::DofMask::GetColumnsFromMatrix
        struct /* GetColumnsFromMatrix */ {
          // Source: drake/planning/dof_mask.h
          const char* doc_2args =
R"""(Gets a subset of the *columns* of ``full_mat`` and writes them to
``output``. Similar to GetFromArray(), but instead of single scalar
values, whole columns are read from ``full_mat`` and written to
``output``.

Precondition:
    ``output`` is not null.

Precondition:
    ``output.cols() == count()``.

Precondition:
    ``full_mat.cols() == size()``.

Precondition:
    ``full_mat.rows() = output->rows()``.)""";
          // Source: drake/planning/dof_mask.h
          const char* doc_1args =
R"""(Overload for GetColumnsFromMatrix() which returns the read values in a
new matrix instead of writing to an output argument.

Precondition:
    ``full_mat.cols() == size()``.)""";
        } GetColumnsFromMatrix;
        // Symbol: drake::planning::DofMask::GetFromArray
        struct /* GetFromArray */ {
          // Source: drake/planning/dof_mask.h
          const char* doc_2args =
R"""(Gets a subset of the values in ``full_vec`` and writes them to
``output``. The values read from ``full_vec`` associated with the
*selected* dofs in ``this`` are written into ``output`` with the same
relative ordering as they appear in ``full_vec``.

For example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {c++}
    // A mask into five total dofs, with indices 0, 1, & 4 selected.
    const DofMask mask({true, true, false, false, true});
    const auto full_vec = (VectorX(5) << 0, 10, 20, 30, 40).finished();
    VectorX selected(mask.count());
    mask.GetFromArray(full_vec, &selected);
    // Print out the vector: [0, 10, 40].
    std∷cout << selected.transpose() << "\n";

.. raw:: html

    </details>

Precondition:
    ``full_vec.size() == size()``.

Precondition:
    ``output`` is not null.

Precondition:
    ``output.size() == count()``.)""";
          // Source: drake/planning/dof_mask.h
          const char* doc_1args =
R"""(Overload for GetFromArray() which returns the read values in a new
vector instead of writing to an output argument.

Precondition:
    ``full_vec.size() == size()``.)""";
        } GetFromArray;
        // Symbol: drake::planning::DofMask::GetFullToSelectedIndex
        struct /* GetFullToSelectedIndex */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(The inverse mapping of GetSelectedToFullIndex(). If we have q_selected
= dof_mask.GetFromArray(q_full), the this function returns the mapping
from q_full index to q_selected index. Namely if dof_mask[i] is true,
namely q_full[i] is selected, then
q_selected[*dof_mask.GetFullToSelectedIndex()[i]] is the same as
q_full[i]; if dof_mask[i] is false, then
dof_mask.GetFullToSelectedIndex()[i] is nullopt.)""";
        } GetFullToSelectedIndex;
        // Symbol: drake::planning::DofMask::GetJoints
        struct /* GetJoints */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(Creates a collection of all of the joints implied by the selected dofs
in ``this``. The returned joint indices are reported in increasing
order.

Precondition:
    ``plant`` is compatible with DofMask.

Precondition:
    ``plant.num_positions() == size()``.)""";
        } GetJoints;
        // Symbol: drake::planning::DofMask::GetSelectedToFullIndex
        struct /* GetSelectedToFullIndex */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(If we have q_selected = dof_mask.GetFromArray(q_full), then this
function returns a mapping from q_selected index to q_full index, such
that q_selected[i] is the same as
q_full[dof_mask.GetSelectedToFullIndex()[i]].)""";
        } GetSelectedToFullIndex;
        // Symbol: drake::planning::DofMask::Intersect
        struct /* Intersect */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(Creates the intersection of ``this`` and ``other``. The result
includes all selected dofs that are in both ``this`` and ``other``.

Precondition:
    ``size() == other.size()``.)""";
        } Intersect;
        // Symbol: drake::planning::DofMask::MakeFromModel
        struct /* MakeFromModel */ {
          // Source: drake/planning/dof_mask.h
          const char* doc_2args_plant_model_index =
R"""(Creates the DofMask associated with a model instance (indicated by
``model_index``) in ``plant``.

Raises:
    RuntimeError if ``plant`` doesn't satisfy the compatibility
    requirements for DofMask.)""";
          // Source: drake/planning/dof_mask.h
          const char* doc_2args_plant_model_name =
R"""(Creates the DofMask associated with the model instance (named by
``model_name``) in ``plant``.

Raises:
    RuntimeError if ``plant`` doesn't satisfy the compatibility
    requirements for DofMask.)""";
        } MakeFromModel;
        // Symbol: drake::planning::DofMask::SetInArray
        struct /* SetInArray */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(Sets the values given in ``vec`` into ``output``. This is the inverse
of GetFromArray(). ``vec`` contains count() values and the iᵗʰ value
in ``vec`` is written to ``output`` at the index corresponding to the
iᵗʰ *selected* dof in ``this``. Entries in ``output`` which correspond
to the *unselected* dofs will remain unchanged.

For example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {c++}
    // A mask into five total dofs, with indices 0, 1, & 4 selected.
    const DofMask mask({true, true, false, false, true});
    const auto dof_vec = (VectorX(3) << -10, -20, -30).finished();
    auto full_vec = (VectorX(5) << 0, 1, 2, 3, 4).finished();
    mask.SetInArray(dof_vec, &full_vec);
    // Print out the vector: [-10, -20, 2, 3, -30].
    std∷cout << full_vec.transpose() << "\n";

.. raw:: html

    </details>

Precondition:
    ``vec.size() = this->count()``.

Precondition:
    ``output`` is not null.

Precondition:
    ``output.size() == size()``.)""";
        } SetInArray;
        // Symbol: drake::planning::DofMask::Subtract
        struct /* Subtract */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(Creates the set difference of ``this`` and ``other``. The result
includes only those selected dofs in ``this`` that are *not* in
``other``.

Precondition:
    ``size() == other.size()``.)""";
        } Subtract;
        // Symbol: drake::planning::DofMask::Union
        struct /* Union */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(Creates the union of ``this`` and ``other``. The result includes all
selected dofs in either ``this`` or ``other``.

Precondition:
    ``size() == other.size()``.)""";
        } Union;
        // Symbol: drake::planning::DofMask::count
        struct /* count */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(Reports this DofMask instance's number of *selected* dofs.)""";
        } count;
        // Symbol: drake::planning::DofMask::operator[]
        struct /* operator_array */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(Precondition:
    ``index`` is in the range [0, size()).)""";
        } operator_array;
        // Symbol: drake::planning::DofMask::size
        struct /* size */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(Reports this DofMask instance's total number of indexable dofs.)""";
        } size;
        // Symbol: drake::planning::DofMask::to_string
        struct /* to_string */ {
          // Source: drake/planning/dof_mask.h
          const char* doc =
R"""(The string representation of the mask -- it encodes the full mask size
clearly indicating which dofs are selected and which are unselected.
The exact format is not guaranteed to remain fixed, but always clear.)""";
        } to_string;
      } DofMask;
      // Symbol: drake::planning::EdgeMeasure
      struct /* EdgeMeasure */ {
        // Source: drake/planning/edge_measure.h
        const char* doc =
R"""(The measure of the distance of the edge from q1 to q2 and the portion
of that is collision free.

Distance is that produced by
CollisionChecker∷ComputeConfigurationDistance() for the entire edge
between q1 and q2.

The portion of the edge between q1 and q2 that is collision free is
encoded as the value α with the following semantics:

- α = 1:
No collisions were detected. The full edge can be considered collision
free. This is the *only* time completely_free() reports ``True``.
- 0 ≤ α < 1:
A collision was detected between q1 and q2. α is the *largest*
interpolation value such that an edge from q1 to qα can be considered
collision free (where qα = interpolate(q1, q2, α)). partially_free()
reports ``True``.
- α is undefined:
q1 was found to be in collision. That means there exists no α for which the
edge (q1, qα) can be collision free.

Note:
    The length of the collision-free edge can be computed via distance
    * α. To simplify comparisons between a number of edges, some of
    which may not have a defined α, the function
    alpha_or(default_value) is provided. This is equivalent to
    ``edge.partially_free() ? edge.alpha() : default_value``.

Note:
    For α to be meaningful, the caller is obliged to make sure that
    they use the same interpolating function as the CollisionChecker
    did when generating the measure. Calling
    CollisionChecker∷InterpolateBetweenConfigurations() on the same
    checker instance would satisfy that requirement.)""";
        // Symbol: drake::planning::EdgeMeasure::EdgeMeasure
        struct /* ctor */ {
          // Source: drake/planning/edge_measure.h
          const char* doc =
R"""(Precondition:
    ``0 ≤ distance``

Precondition:
    ``0 ≤ alpha ≤ 1`` to indicate defined ``alpha``, negative
    otherwise.)""";
        } ctor;
        // Symbol: drake::planning::EdgeMeasure::alpha
        struct /* alpha */ {
          // Source: drake/planning/edge_measure.h
          const char* doc =
R"""(Returns the value of alpha, if defined.

Note: Due to the sampling nature of the edge check, the edge (q1, qα)
may not actually be collision free (due to a missed collision).
There's a further subtlety. Subsequently calling
CheckEdgeCollisionFree(q1, qα) may return ``False``. This apparent
contradiction is due to the fact that the samples on the edge (q1, qα)
will not necessarily be the same as the samples originally tested on
the edge (q1, q2). It is possible for those new samples to detect a
previously missed collision. This is not a bug, merely a property of
sampling-based testing.

Precondition:
    partially_free() returns ``True``.)""";
        } alpha;
        // Symbol: drake::planning::EdgeMeasure::alpha_or
        struct /* alpha_or */ {
          // Source: drake/planning/edge_measure.h
          const char* doc =
R"""(Returns the value of alpha, if defined, or the provided default value.)""";
        } alpha_or;
        // Symbol: drake::planning::EdgeMeasure::completely_free
        struct /* completely_free */ {
          // Source: drake/planning/edge_measure.h
          const char* doc =
R"""(Reports ``True`` if all samples were collision free.)""";
        } completely_free;
        // Symbol: drake::planning::EdgeMeasure::distance
        struct /* distance */ {
          // Source: drake/planning/edge_measure.h
          const char* doc = R"""(Returns the edge distance.)""";
        } distance;
        // Symbol: drake::planning::EdgeMeasure::partially_free
        struct /* partially_free */ {
          // Source: drake/planning/edge_measure.h
          const char* doc =
R"""(Reports ``True`` if there's *any* portion of the edge (starting from
q1) that is collision free. By implication, if completely_free()
reports ``True``, so will this.)""";
        } partially_free;
      } EdgeMeasure;
      // Symbol: drake::planning::JointLimits
      struct /* JointLimits */ {
        // Source: drake/planning/joint_limits.h
        const char* doc =
R"""(Wrapper type for position, velocity, and acceleration limits.

Note that enforcement of finite limits by this class is optional; see
the ``require_finite_*`` constructor arguments.

NaNs are rejected for all limits and tolerances.)""";
        // Symbol: drake::planning::JointLimits::CheckInAccelerationLimits
        struct /* CheckInAccelerationLimits */ {
          // Source: drake/planning/joint_limits.h
          const char* doc =
R"""(Checks if ``acceleration`` is within acceleration limits, relaxed
outward by ``tolerance`` in both directions.

Raises:
    RuntimeError if ``acceleration.size() != num_accelerations()``.

Raises:
    RuntimeError if ``tolerance`` is negative.

Raises:
    RuntimeError if ``acceleration`` or ``tolerance`` contain NaN
    values.)""";
        } CheckInAccelerationLimits;
        // Symbol: drake::planning::JointLimits::CheckInPositionLimits
        struct /* CheckInPositionLimits */ {
          // Source: drake/planning/joint_limits.h
          const char* doc =
R"""(Checks if ``position`` is within position limits, relaxed outward by
``tolerance`` in both directions.

Raises:
    RuntimeError if ``position.size() != num_positions()``.

Raises:
    RuntimeError if ``tolerance`` is negative.

Raises:
    RuntimeError if ``position`` or ``tolerance`` contain NaN values.)""";
        } CheckInPositionLimits;
        // Symbol: drake::planning::JointLimits::CheckInVelocityLimits
        struct /* CheckInVelocityLimits */ {
          // Source: drake/planning/joint_limits.h
          const char* doc =
R"""(Checks if ``velocity`` is within velocity limits, relaxed outward by
``tolerance`` in both directions.

Raises:
    RuntimeError if ``velocity.size() != num_velocities()``.

Raises:
    RuntimeError if ``tolerance`` is negative.

Raises:
    RuntimeError if ``velocity`` or ``tolerance`` contain NaN values.)""";
        } CheckInVelocityLimits;
        // Symbol: drake::planning::JointLimits::JointLimits
        struct /* ctor */ {
          // Source: drake/planning/joint_limits.h
          const char* doc_plant =
R"""(Constructs a JointLimits using the position, velocity, and
acceleration limits in the provided ``plant``.

Raises:
    RuntimeError if plant is not finalized.

Raises:
    RuntimeError if the position, velocity, or acceleration limits
    contain non-finite values, and the corresponding constructor
    argument is true.

Raises:
    RuntimeError if any limit value is NaN.)""";
          // Source: drake/planning/joint_limits.h
          const char* doc_plant_select =
R"""(Constructs a JointLimits using the position, velocity, and
acceleration limits in the provided ``plant``, selecting only the dofs
indicated by ``active_dof.``

Raises:
    RuntimeError if plant is not finalized.

Raises:
    RuntimeError if active_dof.size() != num_positions().

Raises:
    RuntimeError if active_dof.size() != num_velocities().

Raises:
    RuntimeError if active_dof.size() != num_accelerations().

Raises:
    RuntimeError if the position, velocity, or acceleration limits
    contain non-finite values, and the corresponding constructor
    argument is true.

Raises:
    RuntimeError if any limit value is NaN.)""";
          // Source: drake/planning/joint_limits.h
          const char* doc_copy_select =
R"""(Constructs a JointLimits using the position, velocity, and
acceleration limits in the provided ``other``, selecting only the
coefficients indicated by ``active_dof.``

Raises:
    RuntimeError if active_dof.size() != other.num_positions().

Raises:
    RuntimeError if active_dof.size() != other.num_velocities().

Raises:
    RuntimeError if active_dof.size() != other.num_accelerations().

Raises:
    RuntimeError if the position, velocity, or acceleration limits
    contain non-finite values, and the corresponding constructor
    argument is true.

Raises:
    RuntimeError if any limit value is NaN.)""";
          // Source: drake/planning/joint_limits.h
          const char* doc_vectors =
R"""(Constructs a JointLimits using the provided arguments.

Raises:
    RuntimeError if the position, velocity, or acceleration limits
    contain non-finite values, and the corresponding constructor
    argument is true.

Raises:
    RuntimeError if any lower/upper limit pairs differ in size.

Raises:
    RuntimeError if any upper limit coefficients are less than the
    corresponding lower limit coefficient.

Raises:
    RuntimeError if velocity and acceleration limits differ in size.

Raises:
    RuntimeError if any limit value is NaN.)""";
          // Source: drake/planning/joint_limits.h
          const char* doc =
R"""(Constructs a JointLimits with 0-length vectors for all limits.)""";
        } ctor;
        // Symbol: drake::planning::JointLimits::acceleration_lower
        struct /* acceleration_lower */ {
          // Source: drake/planning/joint_limits.h
          const char* doc = R"""()""";
        } acceleration_lower;
        // Symbol: drake::planning::JointLimits::acceleration_upper
        struct /* acceleration_upper */ {
          // Source: drake/planning/joint_limits.h
          const char* doc = R"""()""";
        } acceleration_upper;
        // Symbol: drake::planning::JointLimits::num_accelerations
        struct /* num_accelerations */ {
          // Source: drake/planning/joint_limits.h
          const char* doc = R"""()""";
        } num_accelerations;
        // Symbol: drake::planning::JointLimits::num_positions
        struct /* num_positions */ {
          // Source: drake/planning/joint_limits.h
          const char* doc = R"""()""";
        } num_positions;
        // Symbol: drake::planning::JointLimits::num_velocities
        struct /* num_velocities */ {
          // Source: drake/planning/joint_limits.h
          const char* doc = R"""()""";
        } num_velocities;
        // Symbol: drake::planning::JointLimits::position_lower
        struct /* position_lower */ {
          // Source: drake/planning/joint_limits.h
          const char* doc = R"""()""";
        } position_lower;
        // Symbol: drake::planning::JointLimits::position_upper
        struct /* position_upper */ {
          // Source: drake/planning/joint_limits.h
          const char* doc = R"""()""";
        } position_upper;
        // Symbol: drake::planning::JointLimits::velocity_lower
        struct /* velocity_lower */ {
          // Source: drake/planning/joint_limits.h
          const char* doc = R"""()""";
        } velocity_lower;
        // Symbol: drake::planning::JointLimits::velocity_upper
        struct /* velocity_upper */ {
          // Source: drake/planning/joint_limits.h
          const char* doc = R"""()""";
        } velocity_upper;
      } JointLimits;
      // Symbol: drake::planning::LinearDistanceAndInterpolationProvider
      struct /* LinearDistanceAndInterpolationProvider */ {
        // Source: drake/planning/linear_distance_and_interpolation_provider.h
        const char* doc =
R"""(This class represents a basic "linear" implementation of
DistanceAndInterpolationProvider.

- Configuration distance is computed as difference.cwiseProduct(weights).norm(),
where difference is computed as the angle between quaternion DoF and difference
between all other positions. Default weights are (1, 0, 0, 0) for quaternion
DoF and 1 for all other positions.
- Configuration interpolation is performed using slerp for quaternion DoF and
linear interpolation for all other positions.)""";
        // Symbol: drake::planning::LinearDistanceAndInterpolationProvider::LinearDistanceAndInterpolationProvider
        struct /* ctor */ {
          // Source: drake/planning/linear_distance_and_interpolation_provider.h
          const char* doc_2args_plant_joint_distance_weights =
R"""(Constructs a LinearDistanceAndInterpolationProvider for the specified
``plant`` using the provided map of distance weights
``joint_distance_weights`` and default weights (i.e. 1) for all other
positions.

Precondition:
    all distance weights must be non-negative and finite.)""";
          // Source: drake/planning/linear_distance_and_interpolation_provider.h
          const char* doc_2args_plant_distance_weights =
R"""(Constructs a LinearDistanceAndInterpolationProvider for the specified
``plant`` with the provided distance weights ``distance_weights``.

Precondition:
    distance_weights must be the same size as plant.num_positions(),
    all weights must be non-negative and finite, and weights for
    quaternion DoF must be of the form (weight, 0, 0, 0).)""";
          // Source: drake/planning/linear_distance_and_interpolation_provider.h
          const char* doc_1args_plant =
R"""(Constructs a LinearDistanceAndInterpolationProvider for the specified
``plant`` with default distance weights (i.e. 1) for all positions.
Equivalent to constructing with an empty map of named joint distance
weights.)""";
        } ctor;
        // Symbol: drake::planning::LinearDistanceAndInterpolationProvider::distance_weights
        struct /* distance_weights */ {
          // Source: drake/planning/linear_distance_and_interpolation_provider.h
          const char* doc = R"""(Gets the distance weights.)""";
        } distance_weights;
        // Symbol: drake::planning::LinearDistanceAndInterpolationProvider::quaternion_dof_start_indices
        struct /* quaternion_dof_start_indices */ {
          // Source: drake/planning/linear_distance_and_interpolation_provider.h
          const char* doc =
R"""(Gets the start indices for quaternion DoF in the position vector.)""";
        } quaternion_dof_start_indices;
      } LinearDistanceAndInterpolationProvider;
      // Symbol: drake::planning::MakeBodyShapeDescription
      struct /* MakeBodyShapeDescription */ {
        // Source: drake/planning/body_shape_description.h
        const char* doc =
R"""(Constructs a BodyShapeDescription by extracting the shape, pose, and
names associated with the provided geometry_id.

Precondition:
    ``plant_context`` is compatible with ``plant``.

Precondition:
    ``plant`` is connected to a scene graph.

Precondition:
    ``geometry_id`` refers to a geometry rigidly affixed to a body of
    ``plant``.)""";
      } MakeBodyShapeDescription;
      // Symbol: drake::planning::RobotClearance
      struct /* RobotClearance */ {
        // Source: drake/planning/robot_clearance.h
        const char* doc =
R"""(A summary of the clearance -- a collection of distance measurements --
between the robot and everything in the world. This data can be used
to define collision avoidance strategies.

Conceptually, this class represents a table:

| body index R | body index O | type | ϕᴼ(R) | Jq_ϕᴼ(R) | |
:----------: | :----------: | :--: | :---: | :------: | | ... | ... |
.. | ... | ... |

Member functions return each column of the table as an ordered
collection. The iᵗʰ entry in each collection belongs to the iᵗʰ row.

Each row characterizes the relationship between a particular *robot*
body (annotated as body R) and some "other" body (annotated as body O)
in the model. That other body O may be part of the robot or the
environment.

- ``body index R`` is the BodyIndex of body R.
- ``body index O`` is the BodyIndex of body O.
- ``type`` implies the type of body O. Given that we know body R is a robot
body, ``type`` indicates that body O is also a robot body with the value
RobotCollisionType∷kSelfCollision or an environment body with the value
RobotCollisionType∷kEnvironmentCollision. For a correct implementation of
CollisionChecker, it will never be
RobotCollisionType∷kEnvironmentAndSelfCollision.
- ``ϕᴼ(R)`` is the signed distance function of the other body O evaluated on
body R. The reported distance is offset by the padding value for the body
pair recorded in the CollisionChecker. It is the minimum padded distance
between bodies O and R. A point on the padded surface of body O would
report a distance of zero. Points inside that boundary report negative
distance and points outside have positive distance.
- ``Jqᵣ_ϕᴼ(R)`` is the Jacobian of ϕᴼ(R) with respect to the robot
configuration vector ``qᵣ``. The Jacobian is the derivative as observed in
the world frame.
- The vector ``qᵣ`` will be a subset of the plant's full configuration ``q``
when there are floating bodies or joints in the plant other than the
robot. The Jacobian is only taken with respect to the robot.
- The ``jacobians()`` matrix has ``plant.num_positions()`` columns and the
column order matches up with the full ``plant.GetPositions()`` order.
The columns associated with non-robot joints will be zero.

Several important notes:

- A single robot body index can appear many times as there may be many
measured distances between that robot body and other bodies in the model.
- A row *may* contain a zero-valued Jacobian Jqᵣ_ϕᴼ(R). Appropriately
filtering collisions will cull most of these Jacobians. But depending on
the structure of the robot and its representative collision geometry, it
is still possible to evaluate the Jacobian at a configuration that
represents a local optimum (zero-valued Jacobian).)""";
        // Symbol: drake::planning::RobotClearance::Append
        struct /* Append */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc = R"""(Appends one measurement to this table.)""";
        } Append;
        // Symbol: drake::planning::RobotClearance::Reserve
        struct /* Reserve */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""(Ensures this object has storage for at least ``size`` rows.)""";
        } Reserve;
        // Symbol: drake::planning::RobotClearance::RobotClearance
        struct /* ctor */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""(Creates an empty clearance with size() == 0 and num_positions as
given.)""";
        } ctor;
        // Symbol: drake::planning::RobotClearance::collision_types
        struct /* collision_types */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""(Returns:
    the vector of body collision types.)""";
        } collision_types;
        // Symbol: drake::planning::RobotClearance::distances
        struct /* distances */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""(Returns:
    the vector of distances (``ϕᴼ(R)``).)""";
        } distances;
        // Symbol: drake::planning::RobotClearance::jacobians
        struct /* jacobians */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""(Returns:
    the vector of distance Jacobians (``Jqᵣ_ϕᴼ(R)``); the return type
    is a readonly Eigen∷Map with size() rows and num_positions()
    columns.)""";
        } jacobians;
        // Symbol: drake::planning::RobotClearance::mutable_jacobians
        struct /* mutable_jacobians */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""((Advanced) The mutable flavor of jacobians().)""";
        } mutable_jacobians;
        // Symbol: drake::planning::RobotClearance::num_positions
        struct /* num_positions */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""(Returns:
    the number of positions (i.e., columns) in jacobians().)""";
        } num_positions;
        // Symbol: drake::planning::RobotClearance::other_indices
        struct /* other_indices */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""(Returns:
    the vector of *other* body indices.)""";
        } other_indices;
        // Symbol: drake::planning::RobotClearance::robot_indices
        struct /* robot_indices */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""(Returns:
    the vector of *robot* body indices.)""";
        } robot_indices;
        // Symbol: drake::planning::RobotClearance::size
        struct /* size */ {
          // Source: drake/planning/robot_clearance.h
          const char* doc =
R"""(Returns:
    the number of distance measurements (rows in the table).)""";
        } size;
      } RobotClearance;
      // Symbol: drake::planning::RobotCollisionType
      struct /* RobotCollisionType */ {
        // Source: drake/planning/robot_collision_type.h
        const char* doc =
R"""(Enumerates these predicates (and their combinations): - is the robot
in collision with itself? - is the robot in collision with something
in the environment?)""";
        // Symbol: drake::planning::RobotCollisionType::kEnvironmentAndSelfCollision
        struct /* kEnvironmentAndSelfCollision */ {
          // Source: drake/planning/robot_collision_type.h
          const char* doc = R"""()""";
        } kEnvironmentAndSelfCollision;
        // Symbol: drake::planning::RobotCollisionType::kEnvironmentCollision
        struct /* kEnvironmentCollision */ {
          // Source: drake/planning/robot_collision_type.h
          const char* doc = R"""()""";
        } kEnvironmentCollision;
        // Symbol: drake::planning::RobotCollisionType::kNoCollision
        struct /* kNoCollision */ {
          // Source: drake/planning/robot_collision_type.h
          const char* doc = R"""()""";
        } kNoCollision;
        // Symbol: drake::planning::RobotCollisionType::kSelfCollision
        struct /* kSelfCollision */ {
          // Source: drake/planning/robot_collision_type.h
          const char* doc = R"""()""";
        } kSelfCollision;
      } RobotCollisionType;
      // Symbol: drake::planning::RobotDiagram
      struct /* RobotDiagram */ {
        // Source: drake/planning/robot_diagram.h
        const char* doc =
R"""(Storage for a combined diagram, plant, and scene graph.

This class is a convenient syntactic sugar, especially in C++ code
where it simplifies object lifetime tracking and downcasting of the
plant and scene graph references. It's purpose is to serve as
planner-specific syntactic sugar for operating on a MultibodyPlant.
For other purposes (e.g., simulation), users should generally prefer
to just use a Diagram, instead.

Use RobotDiagramBuilder to construct a RobotDiagram.

By default, the ports exposed by a RobotDiagram are the set of all
ports provided by the plant and scene graph (excluding the internal
connections between the two). Refer to their individual overview
figures for details (see multibody∷MultibodyPlant and
geometry∷SceneGraph), or see the full list by viewing the
robot_diagram.GetGraphvizString().

.. pydrake_system::

    name: RobotDiagram
    input_ports:
    - plant_actuation
    - plant_applied_generalized_force
    - ... etc ...
    output_ports:
    - plant_state
    - ... etc ...
    - scene_graph_query

However, if the RobotDiagramBuilder∷builder() was used to change the
diagram or if either the plant or scene graph were renamed, then no
ports will be exported by default. In that case, you can use the
builder to export any desired ports.)""";
        // Symbol: drake::planning::RobotDiagram::RobotDiagram<T>
        struct /* ctor */ {
          // Source: drake/planning/robot_diagram.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::planning::RobotDiagram::mutable_plant_context
        struct /* mutable_plant_context */ {
          // Source: drake/planning/robot_diagram.h
          const char* doc =
R"""(Gets the contained plant's context (mutable) out of the given root
context. Refer to drake∷systems∷System∷GetMyContextFromRoot() to
understand ``root_context``.

Raises:
    RuntimeError if the ``root_context`` is not a root context.)""";
        } mutable_plant_context;
        // Symbol: drake::planning::RobotDiagram::mutable_scene_graph
        struct /* mutable_scene_graph */ {
          // Source: drake/planning/robot_diagram.h
          const char* doc =
R"""(Gets the contained scene graph (mutable).)""";
        } mutable_scene_graph;
        // Symbol: drake::planning::RobotDiagram::mutable_scene_graph_context
        struct /* mutable_scene_graph_context */ {
          // Source: drake/planning/robot_diagram.h
          const char* doc =
R"""(Gets the contained scene graph's context (mutable) out of the given
root context. Refer to drake∷systems∷System∷GetMyContextFromRoot() to
understand ``root_context``.

Raises:
    RuntimeError if the ``root_context`` is not a root context.)""";
        } mutable_scene_graph_context;
        // Symbol: drake::planning::RobotDiagram::plant
        struct /* plant */ {
          // Source: drake/planning/robot_diagram.h
          const char* doc = R"""(Gets the contained plant (readonly).)""";
        } plant;
        // Symbol: drake::planning::RobotDiagram::plant_context
        struct /* plant_context */ {
          // Source: drake/planning/robot_diagram.h
          const char* doc =
R"""(Gets the contained plant's context (readonly) out of the given root
context. Refer to drake∷systems∷System∷GetMyContextFromRoot() to
understand ``root_context``.

Raises:
    RuntimeError if the ``root_context`` is not a root context.)""";
        } plant_context;
        // Symbol: drake::planning::RobotDiagram::scene_graph
        struct /* scene_graph */ {
          // Source: drake/planning/robot_diagram.h
          const char* doc =
R"""(Gets the contained scene graph (readonly).)""";
        } scene_graph;
        // Symbol: drake::planning::RobotDiagram::scene_graph_context
        struct /* scene_graph_context */ {
          // Source: drake/planning/robot_diagram.h
          const char* doc =
R"""(Gets the contained scene graph's context (readonly) out of the given
root context. Refer to drake∷systems∷System∷GetMyContextFromRoot() to
understand ``root_context``.

Raises:
    RuntimeError if the ``root_context`` is not a root context.)""";
        } scene_graph_context;
      } RobotDiagram;
      // Symbol: drake::planning::RobotDiagramBuilder
      struct /* RobotDiagramBuilder */ {
        // Source: drake/planning/robot_diagram_builder.h
        const char* doc =
R"""(Storage for a combined diagram builder, plant, and scene graph. When T
== double, a parser (and package map) is also available.

This class is a convenient syntactic sugar to help build a robot
diagram, especially in C++ code where it simplifies object lifetime
tracking and downcasting of the plant and scene graph references.)""";
        // Symbol: drake::planning::RobotDiagramBuilder::Build
        struct /* Build */ {
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc =
R"""(Builds the diagram and returns the diagram plus plant and scene graph
in a RobotDiagram. The plant will be finalized during this function,
unless it's already been finalized.

Raises:
    exception when IsDiagramBuilt() already.)""";
        } Build;
        // Symbol: drake::planning::RobotDiagramBuilder::BuildForPython
        struct /* BuildForPython */ {
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc =
R"""((Internal use only) Performs all the actions of Build(). In addition,
transfers ownership of both the built diagram and the internal builder
on return.

Raises:
    exception when IsDiagramBuilt() already.)""";
        } BuildForPython;
        // Symbol: drake::planning::RobotDiagramBuilder::IsDiagramBuilt
        struct /* IsDiagramBuilt */ {
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc =
R"""(Checks if the diagram has already been built.)""";
        } IsDiagramBuilt;
        // Symbol: drake::planning::RobotDiagramBuilder::RobotDiagramBuilder<type-parameter-0-0>
        struct /* ctor */ {
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc =
R"""(Constructs with the specified time step for the contained plant.

Parameter ``time_step``:
    Governs whether the MultibodyPlant is modeled as a discrete system
    (``time_step > 0``) or as a continuous system (``time_step = 0``).
    See multibody_simulation for further details. The default here
    matches the default value from multibody∷MultibodyPlantConfig.)""";
        } ctor;
        // Symbol: drake::planning::RobotDiagramBuilder::builder
        struct /* builder */ {
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc_0args_nonconst =
R"""(Gets the contained DiagramBuilder (mutable). Do not call Build() on
the return value; instead, call Build() on this.

Raises:
    exception when IsDiagramBuilt() already.)""";
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc_0args_const =
R"""(Gets the contained DiagramBuilder (readonly).

Raises:
    exception when IsDiagramBuilt() already.)""";
        } builder;
        // Symbol: drake::planning::RobotDiagramBuilder::parser
        struct /* parser */ {
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc_0args_nonconst =
R"""(Gets the contained Parser (mutable).

Raises:
    exception when IsDiagramBuilt() already.)""";
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc_0args_const =
R"""(Gets the contained Parser (readonly).

Raises:
    exception when IsDiagramBuilt() already.)""";
        } parser;
        // Symbol: drake::planning::RobotDiagramBuilder::plant
        struct /* plant */ {
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc_0args_nonconst =
R"""(Gets the contained plant (mutable).

Raises:
    exception when IsDiagramBuilt() already.)""";
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc_0args_const =
R"""(Gets the contained plant (readonly).

Raises:
    exception when IsDiagramBuilt() already.)""";
        } plant;
        // Symbol: drake::planning::RobotDiagramBuilder::scene_graph
        struct /* scene_graph */ {
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc_0args_nonconst =
R"""(Gets the contained scene graph (mutable).

Raises:
    exception when IsDiagramBuilt() already.)""";
          // Source: drake/planning/robot_diagram_builder.h
          const char* doc_0args_const =
R"""(Gets the contained scene graph (readonly).

Raises:
    exception when IsDiagramBuilt() already.)""";
        } scene_graph;
      } RobotDiagramBuilder;
      // Symbol: drake::planning::SceneGraphCollisionChecker
      struct /* SceneGraphCollisionChecker */ {
        // Source: drake/planning/scene_graph_collision_checker.h
        const char* doc =
R"""(An implementation of CollisionChecker that uses SceneGraph to provide
collision checks. Its behavior and limitations are exactly those of
SceneGraph, e.g., in terms of what kinds of geometries can be
collided.)""";
        // Symbol: drake::planning::SceneGraphCollisionChecker::SceneGraphCollisionChecker
        struct /* ctor */ {
          // Source: drake/planning/scene_graph_collision_checker.h
          const char* doc =
R"""(Creates a new checker with the given params.)""";
        } ctor;
      } SceneGraphCollisionChecker;
      // Symbol: drake::planning::SetInEnvironmentCollision
      struct /* SetInEnvironmentCollision */ {
        // Source: drake/planning/robot_collision_type.h
        const char* doc =
R"""(Returns:
    a RobotCollisionType where the environment-collision value is that
    asserted by ``in_environment_collision``, and the self-collision
    value is that asserted by ``collision_type``.)""";
      } SetInEnvironmentCollision;
      // Symbol: drake::planning::SetInSelfCollision
      struct /* SetInSelfCollision */ {
        // Source: drake/planning/robot_collision_type.h
        const char* doc =
R"""(Returns:
    a RobotCollisionType where the self-collision value is that
    asserted by ``in_self_collision``, and the environment-collision
    value is that asserted by ``collision_type``.)""";
      } SetInSelfCollision;
      // Symbol: drake::planning::UnimplementedCollisionChecker
      struct /* UnimplementedCollisionChecker */ {
        // Source: drake/planning/unimplemented_collision_checker.h
        const char* doc =
R"""(A concrete collision checker implementation that throws an exception
for every virtual function hook. This might be useful for unit testing
or for deriving your own collision checker without providing for the
full suite of operations.)""";
        // Symbol: drake::planning::UnimplementedCollisionChecker::DoAddCollisionShapeToBody
        struct /* DoAddCollisionShapeToBody */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc = R"""()""";
        } DoAddCollisionShapeToBody;
        // Symbol: drake::planning::UnimplementedCollisionChecker::DoCalcContextRobotClearance
        struct /* DoCalcContextRobotClearance */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc = R"""()""";
        } DoCalcContextRobotClearance;
        // Symbol: drake::planning::UnimplementedCollisionChecker::DoCheckContextConfigCollisionFree
        struct /* DoCheckContextConfigCollisionFree */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc = R"""()""";
        } DoCheckContextConfigCollisionFree;
        // Symbol: drake::planning::UnimplementedCollisionChecker::DoClassifyContextBodyCollisions
        struct /* DoClassifyContextBodyCollisions */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc = R"""()""";
        } DoClassifyContextBodyCollisions;
        // Symbol: drake::planning::UnimplementedCollisionChecker::DoClone
        struct /* DoClone */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc = R"""()""";
        } DoClone;
        // Symbol: drake::planning::UnimplementedCollisionChecker::DoMaxContextNumDistances
        struct /* DoMaxContextNumDistances */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc = R"""()""";
        } DoMaxContextNumDistances;
        // Symbol: drake::planning::UnimplementedCollisionChecker::DoUpdateContextPositions
        struct /* DoUpdateContextPositions */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc = R"""()""";
        } DoUpdateContextPositions;
        // Symbol: drake::planning::UnimplementedCollisionChecker::RemoveAddedGeometries
        struct /* RemoveAddedGeometries */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc = R"""()""";
        } RemoveAddedGeometries;
        // Symbol: drake::planning::UnimplementedCollisionChecker::UnimplementedCollisionChecker
        struct /* ctor */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc =
R"""(Constructs a checker.

Parameter ``supports_parallel_checking``:
    will serve as the return value of the
    CollisionChecker∷SupportsParallelChecking() function.)""";
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc_move =
R"""(@name Does not allow copy, move, or assignment.)""";
        } ctor;
        // Symbol: drake::planning::UnimplementedCollisionChecker::UpdateCollisionFilters
        struct /* UpdateCollisionFilters */ {
          // Source: drake/planning/unimplemented_collision_checker.h
          const char* doc = R"""()""";
        } UpdateCollisionFilters;
      } UnimplementedCollisionChecker;
      // Symbol: drake::planning::VisibilityGraph
      struct /* VisibilityGraph */ {
        // Source: drake/planning/visibility_graph.h
        const char* doc =
R"""(Given some number of sampled points in the configuration space of
``checker`'s plant(), computes the "visibility graph" -- two `points``
have an edge between them if the line segment connecting them is
collision free. See CollisionChecker documentation for more
information on how edge collision checks are performed.

Note that this method assumes the collision checker has symmetric
behavior (i.e. checking edge (q1, q2) is the same as checking edge
(q2, q1)). This is true for many collision checkers (e.g. those using
LinearDistanceAndInterpolationProvider, which is the default), but
some more complex spaces with non-linear interpolation (e.g. a Dubin's
car) are not symmetric.

If ``parallelize`` specifies more than one thread, then the
CollisionCheckerParams∷distance_and_interpolation_provider for
``checker`` must be implemented in C++, either by providing the C++
implementation directly directly or by using the default provider.

Returns:
    the adjacency matrix, A(i,j) == true iff points.col(i) is visible
    from points.col(j). A is always symmetric.

Precondition:
    points.rows() == total number of positions in the collision
    checker plant.)""";
      } VisibilityGraph;
    } planning;
  } drake;
} pydrake_doc_planning;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
