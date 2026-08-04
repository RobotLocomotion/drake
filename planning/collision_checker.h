#pragma once

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/parallelism.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/body_shape_description.h"
#include "drake/planning/collision_checker_context.h"
#include "drake/planning/collision_checker_params.h"
#include "drake/planning/distance_and_interpolation_provider.h"
#include "drake/planning/edge_measure.h"
#include "drake/planning/robot_clearance.h"
#include "drake/planning/robot_collision_type.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {

/* Note to developers:

 It is documented that when passing configurations, they must all have finite
 values. However, CollisionChecker relies on other classes to do that validation
 (e.g., MultibodyPlant and DistanceAndInterpolationProvider). This requirement
 is tested in collision_checker_test against regression in those classes. */

/** Interface for collision checkers to use.

 <!-- TODO(SeanCurtis-TRI): This documentation focuses on the context management
  aspect of this class. We need to extend the documentation to give a broader
  overview of the class. The goal is to give sufficient context that they'd know
  what kind of nail this is a hammer for and how to swing it. Topics include,
  but are not limited to:

    - Emphasis that everything is characterized w.r.t. *robot* bodies.
      Distinguish the terms "full model" (robot and environment), "robot" or
      "robot model", and "environment bodies".
    - Performing collision queries
      - individual configurations
      - "edges"
      - Clearance
    - Padding
    - Collision filtering
    - Adding additional collision geometries.
    - Clear guidance on the minimal acts that constitute correct usage. E.g.,
      do I have to call `AllocateContexts()`? When should I not? etc.
    - Clear delineation of serial queries that can be performed in parallel and
      queries that themselves parallelize and guidance on not mixing them.

  It's also important to partition the guidance between "user" documentation and
  "implementer" documentation.
  -->

 This interface builds on the basic multi-threading idiom of Drake: one context
 per thread. It offers two models to achieve multi-threaded parallel collision
 checking:

 - using thread pools (e.g. OpenMP or similar) and "implicit contexts" managed
   by this object
 - using arbitrary threads and "explicit contexts" created by this object

 @anchor ccb_implicit_contexts
 <h5>Implicit Context Parallelism</h5>

 Many methods of this class aren't designed for entry from arbitrary threads
 (e.g. from std::async threads), but rather are designed for use with a main
 thread and various thread-pool-parallel operations achieved by using
 directives like `omp parallel`. To support this usage, the base class
 `AllocateContexts()` protected method establishes a pool of contexts to support
 the implicit context parallelism specified in the constructor. (Note: if the
 collision checker declares that parallel checking is not supported, only one
 implict context will be allocated). `AllocateContexts()` must be called and
 only be called as part of the constructor of a derived class defined as final.

 Once the context pool is created, clients can access a thread-associated
 context by using `model_context(optional<int> context_number)` and related
 methods. These methods may be called in two ways:

 - without a context number, the association between thread and context uses the
   OpenMP notion of thread number
 - with a context number, the method uses the context corresponding to the
   provided number

 Without a context number, these context access methods are only safe under the
 following conditions:

 - the caller is the "main thread"
 - the caller is an OpenMP team thread *during execution of a parallel region*

 With a context number, these context access methods are only safe under the
 following conditions:

 - no two or more threads simultaneously use the same context number

 Methods supporting implicit context parallelism are noted below by having a
 reference to this section; as a rule of thumb, any public method that takes a
 `context_number` argument uses implicit context parallelism.

 Users of this class (derived classes and others) can write their own parallel
 operations using implicit contexts, provided they limit parallel blocks to only
 use `const` methods or methods marked to support implicit contexts parallelism,
 and the parallel operations are:

 - without a context number, only with parallelism using OpenMP directives
 - with a context number, via a parallelization method that provides a notion of
   thread numbers similar in behavior to OpenMP's (i.e. a thread number in
   [0, number of threads), not arbitrary values like
   `std::this_thread::get_id()`)

 To determine the greatest implicit context parallelism that can be achieved in
 a parallelized operation, `GetNumberOfThreads(Parallelism parallelize)`
 returns the lesser of the provided `parallelism` and the supported implicit
 context parallelism.

 <!-- TODO(calderpg-tri, rick-poyner) Add parallel for-loop examples once
      drakelint supports OpenMP pragmas in docs.  -->

 @anchor ccb_explicit_contexts
 <h5>Explicit Context Parallelism</h5>

 It is also possible to use arbitrary thread models to perform collision
 checking in parallel using explicitly created contexts from this class.
 Contexts returned from MakeStandaloneModelContext() may be used in any thread,
 using only `const` methods of this class, or "explicit context" methods.

 Explicit contexts are tracked by this class using `std::weak_ptr` to track
 their lifetimes. This mechanism is used by
 PerformOperationAgainstAllModelContexts to map an operation over all collision
 contexts, whether explicit or implicit.

 Methods supporting explicit context parallelism are noted below by having a
 reference to this section; as a rule of thumb, any public method that takes a
 `model_context` first argument uses explicit context parallelism.

 In practice, multi-threaded collision checking with explicit contexts may look
 something like the example below.

 @code
 const Eigen::VectorXd start_q ...
 const Eigen::VectorXd sample_q1 ...
 const Eigen::VectorXd sample_q2 ...

 const auto check_edge_to = [&collision_checker, &start_q] (
     const Eigen::VectorXd& sample_q,
     CollisionCheckerContext* explicit_context) {
   return collision_checker.CheckContextEdgeCollisionFree(
       explicit_context, start_q, sample_q);
 };

 const auto context_1 = collision_checker.MakeStandaloneModelContext();
 const auto context_2 = collision_checker.MakeStandaloneModelContext();

 auto future_q1 = std::async(std::launch::async, check_edge_to, sample_q1,
                             context_1.get());
 auto future_q2 = std::async(std::launch::async, check_edge_to, sample_q2,
                             context_2.get());

 const double edge_1_valid = future_q1.get();
 const double edge_2_valid = future_q2.get();
 @endcode

 <h5>Mixing Threading Models</h5>

 It is possible to support mixed threading models, i.e., using both OpenMP
 thread pools and arbitrary threads. In this case, each arbitrary thread (say,
 from std::async) should have its own instance of a collision checker made using
 Clone(). Then each arbitrary thread will have its own implicit context pool.

 <h5>Implementing Derived Classes</h5>

 Collision checkers deriving from CollisionChecker *must* support parallel
 operations from both of the above parallelism models. This is generally
 accomplished by placing all mutable state within the per-thread context. If
 this cannot be accomplished, the shared mutable state must be accessed in a
 thread-safe manner. There are APIs that depend on SupportsParallelChecking()
 (e.g., CheckConfigsCollisionFree(), CheckEdgeCollisionFreeParallel(),
 CheckEdgesCollisionFree(), etc); a derived implementation should return `false`
 from SupportsParallelChecking() if there is no meaningful benefit to attempting
 to do work in parallel (e.g., they must fully serialize on shared state).

 @ingroup planning_collision_checker */
class CollisionChecker {
 public:
  // N.B. The copy constructor is protected for use in implementing Clone().
  /** @name     Does not allow copy, move, or assignment. */
  //@{

  CollisionChecker(CollisionChecker&&) = delete;
  CollisionChecker& operator=(const CollisionChecker&) = delete;
  CollisionChecker& operator=(CollisionChecker&&) = delete;
  //@}

  virtual ~CollisionChecker();

  std::unique_ptr<CollisionChecker> Clone() const { return DoClone(); }

  /** @name Robot Model

   These methods all provide access to the underlying robot model's contents. */
  //@{

  /** @returns a const reference to the full model. */
  const RobotDiagram<double>& model() const {
    if (IsInitialSetup()) {
      return *setup_model_;
    } else {
      DRAKE_DEMAND(model_ != nullptr);
      return *model_;
    }
  }

  /** @returns a const reference to the full model's plant. */
  const multibody::MultibodyPlant<double>& plant() const {
    return model().plant();
  }

  /** @returns a const body reference to a body in the full model's plant for
   the given `body_index`. */
  const multibody::RigidBody<double>& get_body(
      multibody::BodyIndex body_index) const {
    return plant().get_body(body_index);
  }

  /** Gets the set of model instances belonging to the robot. The returned
   vector has no duplicates and is in sorted order. */
  const std::vector<multibody::ModelInstanceIndex>& robot_model_instances()
      const {
    return robot_model_instances_;
  }

  /** @returns true if the indicated body is part of the robot. */
  bool IsPartOfRobot(const multibody::RigidBody<double>& body) const;

  /** @returns true if the indicated body is part of the robot. */
  bool IsPartOfRobot(multibody::BodyIndex body_index) const;

  /** @returns a generalized position vector, sized according to the full model,
   whose values are all zero.
   @warning A zero vector is not necessarily a valid configuration, e.g., in
   case the configuration has quaternions, or position constraints, or etc. */
  const Eigen::VectorXd& GetZeroConfiguration() const {
    return zero_configuration_;
  }

  //@}

  /** @name Context management */
  //@{

  /** @returns the number of internal (not standalone) per-thread contexts. */
  int num_allocated_contexts() const { return owned_contexts_.num_contexts(); }

  /** Accesses a collision checking context from within the implicit context
   pool owned by this collision checker.
   @param context_number Optional implicit context number.
   @returns a const reference to either the collision checking context given
   by the `context_number`, or when nullopt the context to be used with the
   current OpenMP thread.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  const CollisionCheckerContext& model_context(
      std::optional<int> context_number = std::nullopt) const;

  // TODO(jeremy.nimmer) This is only lightly used, maybe we should toss it?
  /** Accesses a multibody plant sub-context context from within the implicit
   context pool owned by this collision checker.
   @param context_number Optional implicit context number.
   @returns a const reference to the multibody plant sub-context within
   the context given by the `context_number`, or when nullopt the context to be
   used with the current OpenMP thread.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  const systems::Context<double>& plant_context(
      std::optional<int> context_number = std::nullopt) const {
    return model_context(context_number).plant_context();
  }

  /** Updates the generalized positions `q` in the implicit context specified
   and returns a reference to the MultibodyPlant's now-updated context.
   The implicit context is either that specified by `context_number`, or when
   nullopt the context to be used with the current OpenMP thread.
   @param context_number Optional implicit context number.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism".
   @throws if `q` contains non-finite values. */
  const systems::Context<double>& UpdatePositions(
      const Eigen::VectorXd& q,
      std::optional<int> context_number = std::nullopt) const {
    return UpdateContextPositions(&mutable_model_context(context_number), q);
  }

  /** Explicit Context-based version of UpdatePositions().
   @throws std::exception if `model_context` is `nullptr`.
   @throws if `q` contains non-finite values.
   @see @ref ccb_explicit_contexts "Explicit Context Parallelism". */
  const systems::Context<double>& UpdateContextPositions(
      CollisionCheckerContext* model_context, const Eigen::VectorXd& q) const {
    DRAKE_THROW_UNLESS(model_context != nullptr);
    plant().SetPositions(&model_context->mutable_plant_context(), q);
    DoUpdateContextPositions(model_context);
    return model_context->plant_context();
  }

  /** Make and track a CollisionCheckerContext. The returned context will
   participate in PerformOperationAgainstAllModelContexts() until it is
   destroyed. */
  std::shared_ptr<CollisionCheckerContext> MakeStandaloneModelContext() const;

  /** Allows externally-provided operations that must be performed against all
   contexts in the per-thread context pool, and any standalone contexts made
   with MakeStandaloneModelContext().

   For any standalone contexts, note that it is illegal to mutate a context from
   two different threads. No other threads should be mutating any of our
   standalone contexts when this function is called. */
  void PerformOperationAgainstAllModelContexts(
      const std::function<void(const RobotDiagram<double>&,
                               CollisionCheckerContext*)>& operation);

  //@}

  /** @name Adding geometries to a body

   While the underlying model will have some set of geometries to represent
   each body, it can be useful to extend the representative set of geometries.
   These APIs admit adding and removing such geometries to the underlying model.
   We'll distinguish the geometries added by the checker from those in the
   underlying model by referring to them as "checker" geometries and "model"
   geometries, respectively.

   For example, when an end effector has successfully grasped a manipuland,
   we can add additional geometry to the end effector body, representing the
   extent of the manipuland, causing the motion planning to likewise consider
   collisions with the manipuland. Note, this implicitly treats the manipuland
   as being rigidly affixed to the end effector.

   Checker geometries are managed in groups, identified by "group names". Each
   group can contain one or more checker geometries. This is particularly useful
   when multiple geometries should be added and removed as a whole. Simply by
   storing the shared group name, all checker geometries in the group can be
   removed with a single invocation, relying on the collision checker to do
   the book keeping for you.

   Note that different collision checker implementations may limit the set of
   supported Shape types; e.g. sphere-robot-model collision checkers only add
   sphere shapes to robot bodies, and voxel-based collision checkers cannot add
   individual collision geometries to the environment at all. Therefore, all
   methods for adding collision geometries report if the geometries were added.
   The derived classes should clearly document which shapes they support. */
  //@{

  // TODO(sean.curtis): These BodyShapeDescription APIs are not used. Just cut
  //  them?

  // TODO(sean.curtis): Make these functions [[nodiscard]].

  /** Requests the addition of a shape to a body, both given in `description`.
   If added, the shape will belong to the named geometry group.

   @param group_name    The name of the group to add the geometry to.
   @param description   The data describing the shape and target body.
   @returns `true` if the shape was added. */
  bool AddCollisionShape(const std::string& group_name,
                         const BodyShapeDescription& description);

  // TODO(sean.curtis): Convert this to a simple `bool` return; either all of
  //  the shapes were accepted or none were.
  /** Requests the addition of N shapes to N bodies, each given in the set of
   `descriptions`. Each added shape will belong to the named geometry group.

   @param group_name    The name of the group to add the geometry to.
   @param descriptions  The descriptions of N (shape, body) pairs.
   @returns The total number of shapes in `descriptions` that got added. */
  int AddCollisionShapes(const std::string& group_name,
                         const std::vector<BodyShapeDescription>& descriptions);

  /** Requests the addition of a collection of shapes to bodies across multiple
   geometry groups. `geometry_groups` specifies a collection of (shape, body)
   descriptors across multiple geometry groups.

   @param geometry_groups   A map from a named geometry group to the
                            (shape, body) pairs to add to that group.
   @returns A map from input named geometry group to the *number* of geometries
            added to that group. */
  std::map<std::string, int> AddCollisionShapes(
      const std::map<std::string, std::vector<BodyShapeDescription>>&
          geometry_groups);

  /** Requests the addition of `shape` to the frame A in the checker's model.
   The added `shape` will belong to the named geometry group.

   @param group_name    The name of the group to add the geometry to.
   @param frameA        The frame the shape should be rigidly affixed to.
   @param shape         The requested shape, defined in its canonical frame G.
   @param X_AG          The pose of the shape in the frame A.
   @returns `true` if the shape was added. */
  bool AddCollisionShapeToFrame(const std::string& group_name,
                                const multibody::Frame<double>& frameA,
                                const geometry::Shape& shape,
                                const math::RigidTransform<double>& X_AG);

  /** Requests the addition of `shape` to the body A in the checker's model
   The added `shape` will belong to the named geometry group.

   @param group_name    The name of the group to add the geometry to.
   @param bodyA         The body the shape should be rigidly affixed to.
   @param shape         The requested shape, defined in its canonical frame G.
   @param X_AG          The pose of the shape in body A's frame.
   @returns `true` if the shape was added. */
  bool AddCollisionShapeToBody(const std::string& group_name,
                               const multibody::RigidBody<double>& bodyA,
                               const geometry::Shape& shape,
                               const math::RigidTransform<double>& X_AG);

  /** Gets all checker geometries currently added across the whole checker.
   @returns A mapping from each geometry group name to the collection of
            (shape, body) descriptions in that group. */
  std::map<std::string, std::vector<BodyShapeDescription>>
  GetAllAddedCollisionShapes() const;

  // TODO(sean.curtis): Consider returning the total number of geometries
  //  deleted; here and for "remove all".

  /** Removes all added checker geometries which belong to the named group. */
  void RemoveAllAddedCollisionShapes(const std::string& group_name);

  /** Removes all added checker geometries from all geometry groups. */
  void RemoveAllAddedCollisionShapes();

  //@}

  /** @name Padding the distance between bodies

   Ultimately, the model's bodies are represented with geometries. The distance
   between bodies is the distance between their representative geometries.
   However, the exact distance between those geometries is not necessarily
   helpful in planning. For example,

     - You may want to add padding when real-world objects differ from the
       planning geometry.
     - In some cases, limited penetration is expected, and possibly desirable,
       such as when grasping in clutter.

   Padding is a mechanism where these distances can be "fudged" without
   modifying the underlying model. The *reported* distance between two bodies
   can be decreased by adding *positive* padding or increased by adding
   *negative* padding. One could think of it as padding the geometries -- making
   the objects larger (positive) or smaller (negative). This isn't quite true.
   Padding is defined for *pairs* of geometries.

   Ultimately, the padding data is stored in an NxN matrix (where the underlying
   model has N total bodies). The matrix is symmetric and the entry at (i, j) --
   and (j, i) -- is the padding value to be applied to distance measurements
   between bodies i and j. It is meaningless to apply padding between a
   body and itself. Furthermore, as %CollisionChecker is concerned with the
   state of the *robot*, it is equally meaningless to apply padding between
   *environment* bodies. To avoid the illusion of padding, those entries on
   the diagonal and corresponding to environment body pairs are kept at zero.

   <u>Configuring padding</u>

   <!-- TODO(sean.curtis): We have a function for setting the padding between
    one robot body and *all* environment bodies, but not between a robot body
    and all *other* robot bodies. This apparent hole is only due to the
    judgement that it wouldn't be helpful. If we have a use case that would
    benefit, we can add it. -->

   Padding can be applied at different levels of scope:

     - For all pairs, via SetPaddingMatrix().
     - For all (robot, environment) or (robot, robot) pairs via
       SetPaddingAllRobotEnvironmentPairs() and SetPaddingAllRobotRobotPairs().
     - For all (robot i, environment) pairs via
       SetPaddingOneRobotBodyAllEnvironmentPairs().
     - For pair (body i, body j) via SetPaddingBetween().

   Invocations of these methods make immediate changes to the underlying padding
   data. Changing the order of invocations will change the final padding state.
   In other words, setting padding for a particular pair (A, B) followed by
   calling, for example, SetPaddingOneRobotBodyAllEnvironmentPairs(A) could
   erase the effect of the first invocation.

   @anchor collision_checker_padding_prereqs
   <u>Configuration prerequisites</u>

   In all these configuration methods, there are some specific requirements:

     - The padding value must always be finite.
     - Body indices must be in range (i.e., in [0, N) for a model with N
       total bodies).
     - If the parameters include one or more more body indices, at least one of
       them must reference a robot body.

   <u>Introspection</u>

   The current state of collision padding can be introspected via a number of
   methods:

     - View the underlying NxN padding matrix via GetPaddingMatrix().
     - Find out if padding values are heterogeneous via
       MaybeGetUniformRobotEnvironmentPadding() and
       MaybeGetUniformRobotRobotPadding().
     - Query the specific padding value between a pair of bodies via
       GetPaddingBetween().
     - Find out the maximum padding value via GetLargestPadding().
  */
  //@{

  /** If the padding between all robot bodies and environment bodies is the
   same, returns the common padding value. Returns nullopt otherwise. */
  std::optional<double> MaybeGetUniformRobotEnvironmentPadding() const;

  /** If the padding between all pairs of robot bodies is the same, returns
   the common padding value. Returns nullopt otherwise. */
  std::optional<double> MaybeGetUniformRobotRobotPadding() const;

  /** Gets the padding value for the pair of bodies specified.
   If the body indices are the same, zero will always be returned.
   @throws std::exception if either body index is out of range. */
  double GetPaddingBetween(multibody::BodyIndex bodyA_index,
                           multibody::BodyIndex bodyB_index) const {
    DRAKE_THROW_UNLESS(bodyA_index >= 0 &&
                       bodyA_index < collision_padding_.rows());
    DRAKE_THROW_UNLESS(bodyB_index >= 0 &&
                       bodyB_index < collision_padding_.rows());
    return collision_padding_(int{bodyA_index}, int{bodyB_index});
  }

  /** Overload that uses body references. */
  double GetPaddingBetween(const multibody::RigidBody<double>& bodyA,
                           const multibody::RigidBody<double>& bodyB) const {
    return GetPaddingBetween(bodyA.index(), bodyB.index());
  }

  /** Sets the padding value for the pair of bodies specified.
   @throws std::exception if the @ref collision_checker_padding_prereqs
           "configuration prerequisites" are not met or `bodyA_index ==
           bodyB_index`. */
  void SetPaddingBetween(multibody::BodyIndex bodyA_index,
                         multibody::BodyIndex bodyB_index, double padding);

  /** Overload that uses body references. */
  void SetPaddingBetween(const multibody::RigidBody<double>& bodyA,
                         const multibody::RigidBody<double>& bodyB,
                         double padding) {
    SetPaddingBetween(bodyA.index(), bodyB.index(), padding);
  }

  /** Gets the collision padding matrix. */
  const Eigen::MatrixXd& GetPaddingMatrix() const { return collision_padding_; }

  /** Sets the collision padding matrix. Note that this matrix contains all
   padding data, both robot-robot "self" padding, and robot-environment padding.
   `collision_padding` must have the following properties to be considered
   valid.

     - It is a square NxN matrix (where N is the total number of bodies).
     - Diagonal values are all zero.
     - Entries involving only environment bodies are all zero.
     - It is symmetric.
     - All values are finite.

   @throws std::exception if `collision_padding` doesn't have the enumerated
           properties. */
  void SetPaddingMatrix(const Eigen::MatrixXd& collision_padding);

  /** Gets the current largest collision padding across all (robot, *) body
   pairs. This excludes the meaningless zeros on the diagonal and
   environment-environment pairs; the return value *can* be negative. */
  double GetLargestPadding() const { return max_collision_padding_; }

  /** Sets the environment collision padding for the provided robot body with
   respect to all environment bodies.
   @throws std::exception if the @ref collision_checker_padding_prereqs
           "configuration prerequisites" are not met. */
  void SetPaddingOneRobotBodyAllEnvironmentPairs(
      multibody::BodyIndex body_index, double padding);

  /** Sets the padding for all (robot, environment) pairs.
   @throws std::exception if the @ref collision_checker_padding_prereqs
           "configuration prerequisites" are not met. */
  void SetPaddingAllRobotEnvironmentPairs(double padding);

  /** Sets the padding for all (robot, robot) pairs.
   @throws std::exception if the @ref collision_checker_padding_prereqs
           "configuration prerequisites" are not met. */
  void SetPaddingAllRobotRobotPairs(double padding);

  //@}

  /** @name Collision filtering

   The %CollisionChecker adapts the idea of "collision filtering" to *bodies*
   (see geometry::CollisionFilterManager). In addition to whatever
   collision filters have been declared within the underlying model,
   %CollisionChecker provides mechanisms to layer *additional* filters by
   specifying a pair of bodies as being "filtered". No collisions or
   distance measurements are reported on filtered body pairs.

   The "filter" state of all possible body pairs are stored in a symmetric NxN
   integer-valued matrix, where N is the number of bodies reported by the
   plant owned by this collision checker.

   The matrix can only contain one of three values: 0, 1, and -1. For the
   (i, j) entry in the matrix, each value would be interpreted as follows:

     0: The collision is *not* filtered. Collision checker will report
        collisions and clearance between bodies I and J.
     1: The collision *is* filtered. Collision checker will *not* report
        collisions and clearance between bodies I and J.
    -1: The collision *is* filtered *by definition*. Collision checker will
        *not* report collisions and clearance between bodies I and J and
        the user cannot change this state.

   CollisionChecker limits itself to characterizing the state of the *robot*.
   As such, it *always* filters collisions between pairs of environment
   bodies. It also filters collisions between a body and itself as nonsensical.
   Therefore, the matrix will always have immutable -1s along the diagonal and
   for every cell representing an environment-environment pair.

   The collision filter matrix must remain *consistent*. Only valid values
   for body pairs are accepted. I.e., assigning an (environment, environment)
   pair a value of 0 or 1 is "inconsistent". Likewise doing the same on the
   diagonal. Alternatively, assigning a -1 to any pair with a robot body
   would be inconsistent. The functions for configuring collision filters
   will throw if explicitly asked to make an inconsistent change.

   The %CollisionChecker's definition of filtered body pairs is initially
   drawn from the underlying model's configuration (see
   GetNominalFilteredCollisionMatrix()). However, the CollisionChecker's
   definition of the set of collision filters is independent of the model after
   construction and can be freely modified to allow for greater freedom
   in determining which bodies can affect each other. */
  //@{

  /** Returns the "nominal" collision filter matrix. The nominal matrix is
   initialized at construction time and represents the configuration of the
   model's plant and scene graph. It serves as a reference point to assess
   any changes to collision filters beyond this checker's intrinsic model.

   Collisions between bodies A and B are filtered in the following cases:

     - There exists a welded path between A and B.
     - SceneGraph has filtered the collisions between *all* pairs of geometries
       of A and B.

   Note: SceneGraph allows arbitrary collision filter configuration at the
   *geometry* level. The filters on one geometry of body need not be the same
   as another geometry on the same body. %CollisionChecker is body centric.
   It requires all geometries on a body to be filtered homogeneously. A
   SceneGraph that violates this stricter requirement cannot be used in
   a %CollisionChecker. It is highly unlikely that a SceneGraph instance
   will ever be in this configuration by accident. */
  const Eigen::MatrixXi& GetNominalFilteredCollisionMatrix() const {
    return nominal_filtered_collisions_;
  }

  /** Gets the "active" collision filter matrix. */
  const Eigen::MatrixXi& GetFilteredCollisionMatrix() const {
    return filtered_collisions_;
  }

  /** Sets the "active" collision filter matrix
   @param filter_matrix must meet the above conditions to be a "consistent"
                        collision filter matrix.
   @throws std::exception if the given matrix is incompatible with this
                          collision checker, or if it is inconsistent. */
  void SetCollisionFilterMatrix(const Eigen::MatrixXi& filter_matrix);

  /** Checks if collision is filtered between the two bodies specified.
   Note: collision between two environment bodies is *always* filtered.
   @throws std::exception if either body index is out of range. */
  bool IsCollisionFilteredBetween(multibody::BodyIndex bodyA_index,
                                  multibody::BodyIndex bodyB_index) const;

  /** Overload that uses body references. */
  bool IsCollisionFilteredBetween(
      const multibody::RigidBody<double>& bodyA,
      const multibody::RigidBody<double>& bodyB) const {
    return IsCollisionFilteredBetween(bodyA.index(), bodyB.index());
  }

  /** Declares the body pair (bodyA, bodyB) to be filtered (or not) based on
   `filter_collision`.
   @param filter_collision Sets the to body pair to be filtered if `true`.
   @throws std::exception if either body index is out of range.
   @throws std::exception if both indices refer to the same body.
   @throws std::exception if both indices refer to environment bodies. */
  void SetCollisionFilteredBetween(multibody::BodyIndex bodyA_index,
                                   multibody::BodyIndex bodyB_index,
                                   bool filter_collision);

  /** Overload that uses body references. */
  void SetCollisionFilteredBetween(const multibody::RigidBody<double>& bodyA,
                                   const multibody::RigidBody<double>& bodyB,
                                   bool filter_collision) {
    SetCollisionFilteredBetween(bodyA.index(), bodyB.index(), filter_collision);
  }

  /** Declares that body pair (B, O) is filtered (for all bodies O in this
   checker's plant).
   @throws std::exception if `body_index` is out of range.
   @throws std::exception if `body_index` refers to an environment body. */
  void SetCollisionFilteredWithAllBodies(multibody::BodyIndex body_index);

  /** Overload that uses body references. */
  void SetCollisionFilteredWithAllBodies(
      const multibody::RigidBody<double>& body) {
    SetCollisionFilteredWithAllBodies(body.index());
  }

  //@}

  /** @name Configuration collision checking */
  //@{

  /** Checks a single configuration for collision using the current thread's
   associated context.
   @param q Configuration to check
   @param context_number Optional implicit context number.
   @returns true if collision free, false if in collision.
   @throws if `q` contains non-finite values.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  bool CheckConfigCollisionFree(
      const Eigen::VectorXd& q,
      std::optional<int> context_number = std::nullopt) const;

  /** Explicit Context-based version of CheckConfigCollisionFree().
   @throws std::exception if model_context is nullptr.
   @throws if `q` contains non-finite values.
   @see @ref ccb_explicit_contexts "Explicit Context Parallelism". */
  bool CheckContextConfigCollisionFree(CollisionCheckerContext* model_context,
                                       const Eigen::VectorXd& q) const;

  /** Checks a vector of configurations for collision, evaluating in parallel
   when supported and enabled by `parallelize`. Parallelization in configuration
   collision checks is provided using OpenMP and is supported when both: (1) the
   collision checker declares that parallelization is supported (i.e. when
   SupportsParallelChecking() is true) and (2) when multiple OpenMP threads are
   available for execution.
   See @ref collision_checker_parallel_edge "function-level parallelism" for
   guidance on proper usage.
   @param configs     Configurations to check
   @param parallelize How much should collision checks be parallelized?
   @returns std::vector<uint8_t>, one for each configuration in configs. For
   each configuration, 1 if collision free, 0 if in collision.
   @throws if `configs` contains non-finite values. */
  std::vector<uint8_t> CheckConfigsCollisionFree(
      const std::vector<Eigen::VectorXd>& configs,
      Parallelism parallelize = Parallelism::Max()) const;

  //@}

  /** @name Edge collision checking

   These functions serve motion planning methods, such as sampling-based
   planners and shortcut smoothers, which are concerned about checking that an
   "edge" between two configurations, q1 and q2, is free of collision.

   These functions *approximately* answer the question: is the edge collision
   free. The answer is determined by sampling configurations along the edge
   and reporting the edge to be collision free if all samples are collision
   free. Otherwise, we report the fraction of samples (starting with the start
   configuration) that reported to be collision free. The tested samples include
   the start and end configurations, q1 and q2, respectively.

   %CollisionChecker doesn't know how an edge is defined. An edge can be
   anything from a simple line segment (e.g. a linear path in C-space) to an
   arbitrarily complex path (e.g. a Reeds-Shepp path). The shape of the edge is
   defined implicitly by the ConfigurationInterpolationFunction as an
   interpolation between two configurations `q1` and `q2` (see
   SetConfigurationDistanceFunction() and
   SetConfigurationInterpolationFunction()).

   %CollisionChecker picks configuration samples along an edge by uniformly
   sampling the interpolation *parameter* from zero to one. (By definition, the
   interpolation at zero is equal to `q1` and at one is equal to `q2`.) The
   number of samples is determined by the distance between `q1` and `q2`, as
   provided by a ConfigurationDistanceFunction, divided by "edge step size"
   (see set_edge_step_size()).

   As a result, the selection of interpolation function, distance function, and
   edge step size must be coordinated to ensure that edge collision checking is
   sufficiently accurate for your application.

   <u>Default functions</u>

   The configuration distance function is defined at construction (from
   CollisionCheckerParams). It can be as simple as `|q1 - q2|` or could be
   a weighted norm `|wᵀ⋅(q1 − q2)|` based on joint importance or unit
   reconciliation (e.g., some qs are translational and some are rotational).
   Because of this, the "distance" reported may have arbitrary units.

   Whatever the units are, the edge step size must match. The step size value
   and distance function will determine the number of samples on the edge. The
   smaller the step size, the more samples (and the more expensive the collision
   check becomes).

   If all joints are revolute joints, one reasonable distance function is the
   weighted function `|wᵀ⋅(q1 − q2)|` where the weights are based on joint
   speed. For joint dofs `J = [J₀, J₁, ...]` with corresponding positive maximum
   speeds `[s₀, s₁, ...]`, we identify the speed of the fastest joint
   `sₘₐₓ = maxᵢ(sᵢ)` and define the per-dof weights as `wᵢ = sₘₐₓ / sᵢ`.
   Intuitively, the more time a particular joint requires to cover an angular
   distance, the more significance we attribute to that distance -- it's a
   _time_-biased weighting function. The weights are unitless so the reported
   distance is in radians. For some common arms (IIWA, Panda, UR, Jaco, etc.),
   we have found that an edge step size of 0.05 radians produces reasonable
   results.

   %CollisionChecker has a default interpolation function, as defined by
   MakeDefaultConfigurationInterpolationFunction(). It performs Slerp for
   quaternion-valued dofs and linear interpolation for all other dofs. Note that
   this is not appropriate for all robots, (e.g. those using a BallRpyJoint, or
   any non-holonomic robot). You will need to provide your own interpolation
   function in such cases.

   @anchor collision_checker_parallel_edge
   <u>Function-level parallelism</u>

   Parallelization in some edge collision checks is provided using OpenMP and is
   enabled when both: (1) the collision checker declares that parallelization is
   supported (i.e. when SupportsParallelChecking() is true) and (2) when
   multiple OpenMP threads are available for execution.

   Due to this internal parallelism, special care must be paid when calling
   these methods from any thread that is not the main thread; ensure that, for a
   given collision checker instance, implicit context methods are only called
   from one non-OpenMP thread at a given time.

   <u>Thoughts on configuring %CollisionChecker for edge collision
       detection</u>

   Because the edge collision check samples the edge, there is a perpetual
   trade off between the cost of evaluating the edge and the likelihood that
   a real collision is missed. In practice, the %CollisionChecker should be
   configured to maximize performance for a reasonable level of collision
   detection reliability. There is no definitive method for tuning the
   parameters. Rather than advocating a tuning strategy, we'll simply elaborate
   on the available parameters and leave the actual tuning as an exercise for
   the reader.

   There are two properties that will most directly contribute to the accuracy
   of the edge tests: edge step size and padding. Ultimately, any obstacle in
   _C-space_ whose measure is smaller than the edge step size is likely to be
   missed. The goal is to tune the parameters such that such features -- located
   in an area of interest (i.e., where you want your robot to operate) -- will
   have a low probability of being missed.

   Edge step size is very much a global parameter. It will increase the cost of
   _every_ collision check. If you are unable to anticipate where the small
   features are, a small edge step size will be robust to that uncertainty. As
   such, it serves as a good backstop. It comes at a cost of increasing the cost
   of *every* test, even for large geometries with nothing but coarse features.

   Increasing the padding can likewise reduce the likelihood of missing
   features. Adding padding has the effect of increasing the size of the
   workspace obstacles in C-space. The primary benefit is that it can be applied
   locally. If there is a particular obstacle with fine features that your robot
   will be near, padding between robot and that obstacle can be added so that
   interactions between robot and obstacle are more likely to be caught. Doing
   so leaves the global cost low for coarse features. However, padding comes at
   the cost that physically free edges may no longer be considered free.

   The best tuning will likely include configuring both edge step size and
   applying appropriate padding. */
  //@{

  /** Sets the distance and interpolation provider to use.
   Note that in case any of the (to-be-deprecated) separate distance and
   interpolation functions were in use, this supplants _both_ of them.
   @pre provider satisfies the requirements documents on
   DistanceAndInterpolationProvider.
  */
  void SetDistanceAndInterpolationProvider(
      std::shared_ptr<const DistanceAndInterpolationProvider> provider);

  /** Gets the DistanceAndInterpolationProvider in use. */
  const DistanceAndInterpolationProvider& distance_and_interpolation_provider()
      const {
    return *distance_and_interpolation_provider_;
  }

  /** Sets the configuration distance function to `distance_function`.
   @pre distance_function satisfies the requirements documented on
   ConfigurationDistanceFunction and a DistanceAndInterpolationProvider is not
   already in use.
   @pre the collision checker was created with separate distance and
   interpolation functions, not a combined DistanceAndInterpolationProvider.
   @note the `distance_function` object will be copied and retained by this
   collision checker, so if the function has any lambda-captured data then
   that data must outlive this collision checker. */
  // TODO(calderpg-tri, jwnimmer-tri) Deprecate support for separate distance
  // and interpolation functions.
  void SetConfigurationDistanceFunction(
      const ConfigurationDistanceFunction& distance_function);

  /** Computes configuration-space distance between the provided configurations
   `q1` and `q2`, using the distance function configured at construction-
   time or via SetConfigurationDistanceFunction().
   @throws if `q1` or `q2` contain non-finite values. */
  double ComputeConfigurationDistance(const Eigen::VectorXd& q1,
                                      const Eigen::VectorXd& q2) const {
    return distance_and_interpolation_provider_->ComputeConfigurationDistance(
        q1, q2);
  }

  /** @returns a functor that captures this object, so it can be used like a
   free function. The returned functor is only valid during the lifetime of
   this object. The math of the function is equivalent to
   ComputeConfigurationDistance().
   @warning do not pass this standalone function back into
   SetConfigurationDistanceFunction() function; doing so would create an
   infinite loop.
   @throws if `q1` or `q2` contain non-finite values. */
  ConfigurationDistanceFunction MakeStandaloneConfigurationDistanceFunction()
      const;

  /** Sets the configuration interpolation function to `interpolation_function`.

   @param interpolation_function a functor, or nullptr. If nullptr, the default
   function will be configured and used.
   @pre interpolation_function satisfies the requirements documented on
   ConfigurationInterpolationFunction, or is nullptr and a
   DistanceAndInterpolationProvider is not already in use.
   @pre the collision checker was created with separate distance and
   interpolation functions, not a combined DistanceAndInterpolationProvider.
   @note the `interpolation_function` object will be copied and retained by
   this collision checker, so if the function has any lambda-captured data
   then that data must outlive this collision checker.
   @note the default function uses linear interpolation for most variables,
   and uses slerp for quaternion valued variables.*/
  // TODO(calderpg-tri, jwnimmer-tri) Deprecate support for separate distance
  // and interpolation functions.
  void SetConfigurationInterpolationFunction(
      const ConfigurationInterpolationFunction& interpolation_function);

  /** Interpolates between provided configurations `q1` and `q2`.
   @param ratio Interpolation ratio.
   @returns Interpolated configuration.
   @throws std::exception if ratio is not in range [0, 1].
   @throws if `q1` or `q2` contain non-finite values.
   @see ConfigurationInterpolationFunction for more. */
  Eigen::VectorXd InterpolateBetweenConfigurations(const Eigen::VectorXd& q1,
                                                   const Eigen::VectorXd& q2,
                                                   double ratio) const {
    return distance_and_interpolation_provider_
        ->InterpolateBetweenConfigurations(q1, q2, ratio);
  }

  /** @returns a functor that captures this object, so it can be used like a
   free function. The returned functor is only valid during the lifetime of
   this object. The math of the function is equivalent to
   InterpolateBetweenConfigurations().
   @warning do not pass this standalone function back into our
   SetConfigurationInterpolationFunction() function; doing so would create
   an infinite loop. */
  ConfigurationInterpolationFunction
  MakeStandaloneConfigurationInterpolationFunction() const;

  /** Gets the current edge step size. */
  double edge_step_size() const { return edge_step_size_; }

  /** Sets the edge step size to `edge_step_size`.
   @throws std::exception if `edge_step_size` is not positive. */
  void set_edge_step_size(double edge_step_size) {
    DRAKE_THROW_UNLESS(edge_step_size > 0.0);
    edge_step_size_ = edge_step_size;
  }

  /** Checks a single configuration-to-configuration edge for collision, using
   the current thread's associated context.
   @param q1 Start configuration for edge.
   @param q2 End configuration for edge.
   @param context_number Optional implicit context number.
   @returns true if collision free, false if in collision.
   @throws if `q1` or `q2` contain non-finite values.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  bool CheckEdgeCollisionFree(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
      std::optional<int> context_number = std::nullopt) const;

  /** Explicit Context-based version of CheckEdgeCollisionFree().
   @throws if `q1` or `q2` contain non-finite values.
   @throws std::exception if `model_context` is nullptr.
   @see @ref ccb_explicit_contexts "Explicit Context Parallelism". */
  bool CheckContextEdgeCollisionFree(CollisionCheckerContext* model_context,
                                     const Eigen::VectorXd& q1,
                                     const Eigen::VectorXd& q2) const;

  /** Checks a single configuration-to-configuration edge for collision.
   Collision check is parallelized via OpenMP when supported.
   See @ref collision_checker_parallel_edge "function-level parallelism" for
   guidance on proper usage.
   @param q1 Start configuration for edge.
   @param q2 End configuration for edge.
   @param parallelize How much should edge collision check be parallelized?
   @returns true if collision free, false if in collision.
   @throws if `q1` or `q2` contain non-finite values. */
  bool CheckEdgeCollisionFreeParallel(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
      Parallelism parallelize = Parallelism::Max()) const;

  /** Checks multiple configuration-to-configuration edges for collision.
   Collision checks are parallelized via OpenMP when supported and enabled by
   `parallelize`.
   See @ref collision_checker_parallel_edge "function-level parallelism" for
   guidance on proper usage.
   @param edges        Edges to check, each in the form of pair<q1, q2>.
   @param parallelize  How much should edge collision checks be parallelized?
   @returns std::vector<uint8_t>, one for each edge in edges. For each edge, 1
   if collision free, 0 if in collision.
   @throws if any vector in `edges` contains non-finite values. */
  std::vector<uint8_t> CheckEdgesCollisionFree(
      const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges,
      Parallelism parallelize = Parallelism::Max()) const;

  /** Checks a single configuration-to-configuration edge for collision, using
   the current thread's associated context.
   @param q1 Start configuration for edge.
   @param q2 End configuration for edge.
   @param context_number Optional implicit context number.
   @returns A measure of how much of the edge is collision free.
   @throws if `q1` or `q2` contain non-finite values.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  EdgeMeasure MeasureEdgeCollisionFree(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
      std::optional<int> context_number = std::nullopt) const;

  /** Explicit Context-based version of MeasureEdgeCollisionFree().
   @throws std::exception if `model_context` is nullptr.
   @throws if `q1` or `q2` contain non-finite values.
   @see @ref ccb_explicit_contexts "Explicit Context Parallelism". */
  EdgeMeasure MeasureContextEdgeCollisionFree(
      CollisionCheckerContext* model_context, const Eigen::VectorXd& q1,
      const Eigen::VectorXd& q2) const;

  /** Checks a single configuration-to-configuration edge for collision.
   Collision check is parallelized via OpenMP when supported.
   See @ref collision_checker_parallel_edge "function-level parallelism" for
   guidance on proper usage.
   @param q1 Start configuration for edge.
   @param q2 End configuration for edge.
   @param parallelize How much should edge collision check be parallelized?
   @returns A measure of how much of the edge is collision free.
   @throws if `q1` or `q2` contain non-finite values. */
  EdgeMeasure MeasureEdgeCollisionFreeParallel(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
      Parallelism parallelize = Parallelism::Max()) const;

  /** Checks multiple configuration-to-configuration edge for collision.
   Collision checks are parallelized via OpenMP when supported and enabled by
   `parallelize`.
   See @ref collision_checker_parallel_edge "function-level parallelism" for
   guidance on proper usage.
   @param edges        Edges to check, each in the form of pair<q1, q2>.
   @param parallelize  How much should edge collision checks be parallelized?
   @returns A measure of how much of each edge is collision free. The iᵗʰ entry
            is the result for the iᵗʰ edge.
   @throws if any vector in `edges` contains non-finite values. */
  std::vector<EdgeMeasure> MeasureEdgesCollisionFree(
      const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges,
      Parallelism parallelize = Parallelism::Max()) const;

  //@}

  /** @name Robot collision state

   These methods help characterize the robot's collision state with respect to
   a particular robot configuration.

   In this section, the "collision state" is characterized with two different
   APIs:

      - "clearance", a measure of how near to collision the robot as computed by
         CalcRobotClearance(), and
      - a boolean colliding state for each robot body as computed by
        ClassifyBodyCollisions(). */
  //@{

  /** Calculates the distance, ϕ, and distance Jacobian, Jqᵣ_ϕ, for each
   potential collision whose distance is less than `influence_distance`,
   using the current thread's associated context.

   Distances for filtered collisions will not be returned.

   Distances between a pair of robot bodies (i.e., where `collision_types()`
   reports `SelfCollision`) report one body's index in `robot_indices()` and the
   the other body's in `other_indices()`; which body appears in which column is
   arbitrary.

   The total number of rows can depend on how the model is defined and how a
   particular CollisionChecker instance is implemented (see MaxNumDistances()).
   @see RobotClearance for details on the quantities ϕ and Jqᵣ_ϕ (and other
   details).
   @param context_number Optional implicit context number.
   @throws if `q` contains non-finite values.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  RobotClearance CalcRobotClearance(
      const Eigen::VectorXd& q, double influence_distance,
      std::optional<int> context_number = std::nullopt) const;

  /** Explicit Context-based version of CalcRobotClearance().
   @throws std::exception if `model_context` is nullptr.
   @throws if `q` contains non-finite values.
   @see @ref ccb_explicit_contexts "Explicit Context Parallelism". */
  RobotClearance CalcContextRobotClearance(
      CollisionCheckerContext* model_context, const Eigen::VectorXd& q,
      double influence_distance) const;

  // TODO(calderpg-tri) Improve MaxNumDistances to use the prototype context
  // instead, and deprecate context-specific forms.
  /** Returns an upper bound on the number of distances returned by
   CalcRobotClearance(), using the current thread's associated context.
   @param context_number Optional implicit context number.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  int MaxNumDistances(std::optional<int> context_number = std::nullopt) const;

  /** Explicit Context-based version of MaxNumDistances().
   @see @ref ccb_explicit_contexts "Explicit Context Parallelism". */
  int MaxContextNumDistances(
      const CollisionCheckerContext& model_context) const;

  /** Classifies which robot bodies are in collision (and which type of
   collision) for the provided configuration `q`, using the current thread's
   associated context.
   @param context_number Optional implicit context number.
   @returns a vector of collision types arranged in body index order. Only
   entries for robot bodies are guaranteed to be valid; entries for
   environment bodies are populated with kNoCollision, regardless of their
   actual status.
   @throws if `q` contains non-finite values.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  std::vector<RobotCollisionType> ClassifyBodyCollisions(
      const Eigen::VectorXd& q,
      std::optional<int> context_number = std::nullopt) const;

  /** Explicit Context-based version of ClassifyBodyCollisions().
   @throws std::exception if `model_context` is nullptr.
   @throws if `q` contains non-finite values.
   @see @ref ccb_explicit_contexts "Explicit Context Parallelism". */
  std::vector<RobotCollisionType> ClassifyContextBodyCollisions(
      CollisionCheckerContext* model_context, const Eigen::VectorXd& q) const;

  //@}

  /** Does the collision checker support true parallel collision checks?
   @returns true if parallel checking is supported. */
  bool SupportsParallelChecking() const { return supports_parallel_checking_; }

 protected:
  /** Derived classes declare upon construction whether they support parallel
   checking (see SupportsParallelChecking()). If a derived class does not
   support parallel checking, it must set params.implicit_context_parallelism to
   Parallelism::None(); otherwise this constructor will throw.
   @throws std::exception if params is invalid. @see CollisionCheckerParams.
   */
  CollisionChecker(CollisionCheckerParams params,
                   bool supports_parallel_checking);

  /** To support Clone(), allow copying (but not move nor assign). */
  CollisionChecker(const CollisionChecker&);

  /** Allocate the per-thread context pool, and discontinue mutable access to
   the robot model. This must be called and only be called as part of the
   constructor in a derived class defined as final.

   @pre This cannot have already been called for this instance. */
  void AllocateContexts();

  /** Collision checkers that use derived context types can override this
   implementation to allocate their context type instead. */
  virtual std::unique_ptr<CollisionCheckerContext> CreatePrototypeContext()
      const {
    return std::make_unique<CollisionCheckerContext>(&model());
  }

  /** @returns true if called during initial setup (before AllocateContexts()
   is called). */
  bool IsInitialSetup() const { return setup_model_ != nullptr; }

  /** @returns a mutable reference to the robot model.
      @throws std::exception if IsInitialSetup() == false. */
  RobotDiagram<double>& GetMutableSetupModel() {
    DRAKE_THROW_UNLESS(IsInitialSetup());
    return *setup_model_;
  }

  /** @name Internal overridable implementations of public methods. */
  //@{

  /** Derived collision checkers implement can make use of the protected copy
   constructor to implement DoClone(). */
  virtual std::unique_ptr<CollisionChecker> DoClone() const = 0;

  /** Derived collision checkers can do further work in this function in
   response to updates to the MultibodyPlant positions. CollisionChecker
   guarantees that `model_context` will not be nullptr and that the new
   positions are present in model_context->plant_context(). */
  virtual void DoUpdateContextPositions(
      CollisionCheckerContext* model_context) const = 0;

  /** Derived collision checkers are responsible for reporting the collision
   status of the configuration. CollisionChecker guarantees that the passed
   `model_context` has been updated with the configuration `q` supplied to the
   public method. */
  virtual bool DoCheckContextConfigCollisionFree(
      const CollisionCheckerContext& model_context) const = 0;

  /** Does the work of adding a shape to be rigidly affixed to the body. Derived
   checkers can choose to ignore the request, but must return `nullopt` if they
   do so. */
  virtual std::optional<geometry::GeometryId> DoAddCollisionShapeToBody(
      const std::string& group_name, const multibody::RigidBody<double>& bodyA,
      const geometry::Shape& shape,
      const math::RigidTransform<double>& X_AG) = 0;

  /** Representation of an "added" shape. These are shapes that get added to
   the model via the CollisionChecker's Shape API. They encode the id for the
   added geometry and the index of the body (robot or environment) to which the
   geometry is affixed. */
  struct AddedShape {
    /** The id of the geometry. */
    geometry::GeometryId geometry_id;

    /** The index of the body the shape was added; could be robot or
     environment. */
    multibody::BodyIndex body_index;

    /** The full body description.
     We have the invariant that `body_index` has the body and model instance
     names recorded in the description. */
    BodyShapeDescription description;
  };

  /** Removes all of the given added shapes (if they exist) from the checker. */
  virtual void RemoveAddedGeometries(const std::vector<AddedShape>& shapes) = 0;

  /** Derived collision checkers can do further work in this function in
   response to changes in collision filters. This is called after any changes
   are made to the collision filter matrix. */
  virtual void UpdateCollisionFilters() = 0;

  /** Derived collision checkers are responsible for defining the reported
   measurements. But they must adhere to the characteristics documented on
   RobotClearance, e.g., one measurement per row. CollisionChecker guarantees
   that `influence_distance` is finite and non-negative. */
  virtual RobotClearance DoCalcContextRobotClearance(
      const CollisionCheckerContext& model_context,
      double influence_distance) const = 0;

  /** Derived collision checkers are responsible for choosing a collision type
   for each of the robot bodies. They should adhere to the semantics documented
   for ClassifyBodyCollisions. CollisionChecker guarantees that the passed
   `model_context` has been updated with the configuration `q` supplied to the
   public method. */
  virtual std::vector<RobotCollisionType> DoClassifyContextBodyCollisions(
      const CollisionCheckerContext& model_context) const = 0;

  /** Derived collision checkers must implement the semantics documented for
   MaxNumDistances. CollisionChecker does nothing; it just calls this method. */
  virtual int DoMaxContextNumDistances(
      const CollisionCheckerContext& model_context) const = 0;

  //@}

  /** @returns true if this object SupportsParallelChecking() and more than one
   thread is available. */
  bool CanEvaluateInParallel() const;

  /* (Testing only.) Checks that the padding matrix in this instance is valid,
   returning an error message if problems are found, or an empty string if not.
   */
  std::string CriticizePaddingMatrix() const;

 private:
  /* @param context_number Optional implicit context number.
   @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  CollisionCheckerContext& mutable_model_context(
      std::optional<int> context_number) const;

  /* Tests the given filtered collision matrix for several invariants, throwing
   if they are not satisfied:

      - The matrix is square.
      - The diagonal is all -1.
      - The matrix is symmetric.
      - For off-diagonal, non-zero values:
          - 1: at least one of the bodies in the pair is a robot body
          - -1: both bodies in the pair are environment bodies.
      - All entries are either 0, 1, or -1. */
  void ValidateFilteredCollisionMatrix(const Eigen::MatrixXi& filtered,
                                       const char* func) const;

  /* The "nominal" collision matrix. This is intended to be called only upon
   construction.

   The nominal filtered collision matrix has the following properties:

      - All diagonal entries are set to -1 (a body colliding with itself
        is meaningless).
      - All environment-environment pairs are set to -1. CollisionChecker
        ignores them; so it might as well be explicit about it in the matrix.
      - For bodies I and J,
        - if all geometries of I have "filtered collisions" (in the SceneGraph
          sense), with all the geometries of J, the matrix is set to 1.
        - If *no* geometries of I and J are filtered, the matrix is set to 0.
        - If some geometry pairs are filtered, and some are not, an error is
          thrown.
      - If a welded path exists between two bodies (in the MultibodyPlant),
        the matrix is set to 1. */
  Eigen::MatrixXi GenerateFilteredCollisionMatrix() const;

  /* Updates the stored value representing the largest value found in the
   padding matrix -- this excludes the meaningless zeros on the diagonal. */
  void UpdateMaxCollisionPadding();

  /* Tests the given collision padding matrix for several invariants, throwing
   if they are not satisfied:

      - It is a square NxN matrix (where N is the total number of bodies).
      - Diagonal values are all zero.
      - Entries involving only environment bodies are all zero.
      - It is symmetric.
      - All values are finite. */
  void ValidatePaddingMatrix(const Eigen::MatrixXd& padding,
                             const char* func) const;

  /* Checks the same conditions as ValidatePaddingMatrix, but instead of
   throwing, returns the error message, or an empty string if no errors were
   found. */
  std::string CriticizePaddingMatrix(const Eigen::MatrixXd& padding,
                                     const char* func) const;

  /* Gets the number of threads that may be used in an OpenMP-parallelized
   loop, which is the lesser of (a) the number of implicit contexts or (b) the
   number of threads specified by `parallelize`. If OpenMP is not available,
   returns 1. */
  int GetNumberOfThreads(Parallelism parallelize) const;

  /* @returns a generalized position vector, sized according to the full model,
   whose values come from the plant's default context. */
  const Eigen::VectorXd& GetDefaultConfiguration() const {
    return default_configuration_;
  }

  /* This class allocates and maintains the implicit context pool. When the
   CollisionChecker is evaluated in its implicit mode, the contexts used are
   drawn from this collection, either by a `context_number` or by the OpenMP
   thread number associated with the context.

   In addition, this container takes ownership of a reference "prototype"
   context (see below for details about the prototype context).

   @note this class has two phases: `empty()` (before calling
   AllocateOwnedContexts()), and `allocated()`. In the `empty()` phase,
   `num_contexts()` returns 0, and all methods that access a context will
   throw. */
  class OwnedContextKeeper {
   public:
    OwnedContextKeeper() {}

    ~OwnedContextKeeper();

    /* The copy constructor is used to implement our outer class's Clone(). */
    explicit OwnedContextKeeper(const OwnedContextKeeper& other);

    /* Does not allow assignment. */
    void operator=(const OwnedContextKeeper&) = delete;

    /* Allocates the requested number of contexts, for later access by index,
     and clones the passed prototype context.
     @note This method is exposed since
     CollisionChecker::AllocateContexts can't be called in the
     CollisionChecker constructor.
     @throws std::exception if called more than once. */
    void AllocateOwnedContexts(const CollisionCheckerContext& prototype_context,
                               int num_contexts);

    /* @returns true if the keeper is empty. In the empty state, there are no
     contexts allocated and no prototype context is available. */
    bool empty() const {
      DRAKE_DEMAND((prototype_context_ == nullptr) == model_contexts_.empty());
      return model_contexts_.empty();
    }

    /* @returns true if the keeper has allocated contexts. In the allocated
     state, there are contexts available for access by index, and the prototype
     context is available. */
    bool allocated() const { return !empty(); }

    int num_contexts() const { return model_contexts_.size(); }

    /* Gets the special "prototype" context used for copy & clone operations. */
    const CollisionCheckerContext& prototype_context() const {
      // The prototype context is only available in the allocated phase.
      DRAKE_THROW_UNLESS(allocated());
      return *prototype_context_;
    }

    CollisionCheckerContext& get_mutable_model_context(int index) const {
      return *model_contexts_.at(index);
    }

    const CollisionCheckerContext& get_model_context(int index) const {
      return *model_contexts_.at(index);
    }

    /* Performs the provided `operation` on all owned contexts.
     @pre operation != nullptr; */
    void PerformOperationAgainstAllOwnedContexts(
        const RobotDiagram<double>& model,
        const std::function<void(const RobotDiagram<double>&,
                                 CollisionCheckerContext*)>& operation);

   private:
    std::vector<std::unique_ptr<CollisionCheckerContext>> model_contexts_;

    /* This prototype context plays an important role. The CollisionChecker is
     supposed to allow the execution of *any* const methods in parallel without
     incident. This prototype is vital to include Clone() in that set.

     The various const query methods (e.g., CheckConfigCollisionFree()) treat
     the underlying per-thread context as being functionally mutable. To
     evaluate the query, the positions in the MbP's context must be updated to
     the given configuration values. Therefore, if we were to attempt to clone
     this checker while such a query is being evaluated, we *must* not read from
     that thread's Context so we don't catch it in an intermediate state.

     Instead, we clone the collision checker's contexts by cloning this
     prototype context (which is not accessed by any other const method)
     repeatedly. For this approach to be valid, we maintain the invariant that
     this prototype context is bit-identical with all of the owned and
     standalone contexts for this CollisionChecker (except for those differences
     directly attributable to setting the qs in MbP). */
    std::unique_ptr<CollisionCheckerContext> prototype_context_;
  };

  /* When using explicit context parallelism (e.g. when using a parallelization
   system that does not have a strong concept of "thread number" like OpenMP),
   users of CollisionChecker must request "standalone" contexts. This keeps a
   *weak* reference to each of the allocated standalone contexts so that they
   can be updated in lock step with all other contexts in response to
   PerformOperationAgainstAllModelContexts().

   The references are weak references so that the user has full control over
   the lifespan of the standalone contexts. */
  class StandaloneContextReferenceKeeper {
   public:
    StandaloneContextReferenceKeeper() {}

    ~StandaloneContextReferenceKeeper();

    /* The copy constructor is used to implement our outer class's Clone(). */
    explicit StandaloneContextReferenceKeeper(
        const StandaloneContextReferenceKeeper&) {
      // Nothing to do here; contexts should NOT be copied during Clone().
    }

    /* Does not allow assignment. */
    void operator=(const StandaloneContextReferenceKeeper&) = delete;

    void AddStandaloneContext(const std::shared_ptr<CollisionCheckerContext>&
                                  standalone_context) const;

    /* Performs the provided `operation` on all live referenced contexts, and
     removes any references to dead contexts.
     @pre operation != nullptr; */
    void PerformOperationAgainstAllStandaloneContexts(
        const RobotDiagram<double>& model,
        const std::function<void(const RobotDiagram<double>&,
                                 CollisionCheckerContext*)>& operation);

   private:
    mutable std::list<std::weak_ptr<CollisionCheckerContext>>
        standalone_contexts_;
    mutable std::mutex standalone_contexts_mutex_;
  };

  /* Model of the robot used during initial setup only. */
  std::shared_ptr<RobotDiagram<double>> setup_model_;

  /* Model of the robot to perform collision checks with. */
  std::shared_ptr<const RobotDiagram<double>> model_;

  /* We maintain per-thread contexts to allow multiple simultaneous queries. */
  OwnedContextKeeper owned_contexts_;

  /* We maintain weak references to standalone contexts to allow updates to
   standalone contexts. These must be weak refs to avoid keeping standalone
   contexts alive indefinitely. */
  StandaloneContextReferenceKeeper standalone_contexts_;

  /* We maintain a set of all robot model instances for lookups. This vector is
   already de-duplicated and sorted. */
  const std::vector<multibody::ModelInstanceIndex> robot_model_instances_;

  /* The set of indices in q that kinematically affect the robot_model_instances
   but which are not themselves part of robot_model_instances (e.g., a mobile
   base). In many cases this vector will be empty. */
  const std::vector<int> uncontrolled_dofs_that_kinematically_affect_the_robot_;

  /* Provider for distance and interpolation functions */
  std::shared_ptr<const DistanceAndInterpolationProvider>
      distance_and_interpolation_provider_;

  /* Step size for edge collision checking. */
  double edge_step_size_ = 0.0;

  /* Storage for body-body collision padding. */
  Eigen::MatrixXd collision_padding_;

  /* The current maximum collision padding. */
  double max_collision_padding_ = 0.0;

  /* Internal storage of the filtered collision matrix. */
  Eigen::MatrixXi filtered_collisions_;

  /* Internal storage of the nominal filtered collision matrix generated at
   setup time. */
  Eigen::MatrixXi nominal_filtered_collisions_;

  /* We maintain a "zero configuration" of the model. */
  Eigen::VectorXd zero_configuration_;

  /* We maintain a "default configuration" of the model (based on the plant's
   default allocated context). */
  Eigen::VectorXd default_configuration_;

  /* Determines whether the checker reports support for parallel evaluation.
   This is defined upon construction by implementations. */
  bool supports_parallel_checking_{};

  /* Specifies how much parallelism can be used in implicit context operations.
  If the checker does not support parallel evaluation, this will specify no
  parallelism. */
  Parallelism implicit_context_parallelism_;

  /* The names of all groups with added geometries. */
  std::map<std::string, std::vector<AddedShape>> geometry_groups_;
};

}  // namespace planning
}  // namespace drake
