#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace geometry {
namespace optimization {

/** Configuration options for the IRIS algorithm.

@ingroup geometry_optimization
*/
struct IrisOptions {
  IrisOptions() = default;

  /** The initial polytope is guaranteed to contain the point if that point is
  collision-free. However, the IRIS alternation objectives do not include (and
  can not easily include) a constraint that the original sample point is
  contained. Therefore, the IRIS paper recommends that if containment is a
  requirement, then the algorithm should simply terminate early if alternations
  would ever cause the set to not contain the point. */
  bool require_sample_point_is_contained{false};

  /** Maximum number of iterations. */
  int iteration_limit{100};

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this threshold. This termination condition can
  be disabled by setting to a negative value. */
  double termination_threshold{2e-2};  // from rdeits/iris-distro.

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this percent of the previouse best volume.
  This termination condition can be disabled by setting to a negative value. */
  double relative_termination_threshold{1e-3};  // from rdeits/iris-distro.

  // TODO(russt): Improve the implementation so that we can clearly document the
  // units for this margin.
  /** For IRIS in configuration space, we retreat by this margin from each
  C-space obstacle in order to avoid the possibility of requiring an infinite
  number of faces to approximate a curved boundary.
  */
  double configuration_space_margin{1e-2};

  /** For each possible collision, IRIS will search for a counter-example by
  formulating a (likely nonconvex) optimization problem. The initial guess for
  this optimization is taken by sampling uniformly inside the current IRIS
  region. This option controls the termination condition for that
  counter-example search, defining the number of consecutive failures to find a
  counter-example requested before moving on to the next constraint. */
  int num_collision_infeasible_samples{5};

  /** For IRIS in configuration space, it can be beneficial to not only specify
  task-space obstacles (passed in through the plant) but also obstacles that are
  defined by convex sets in the configuration space. This option can be used to
  pass in such configuration space obstacles. */
  ConvexSets configuration_obstacles{};

  /** By default, IRIS in configuration space certifies regions for collision
  avoidance constraints and joint limits. This option can be used to pass
  additional constraints that should be satisfied by the IRIS region. We accept
  these in the form of a MathematicalProgram:

    find q subject to g(q) ≤ 0.

  The decision_variables() for the program are taken to define `q`. IRIS will
  silently ignore any costs in `prog_with_additional_constraints`, and will
  throw std::runtime_error if it contains any unsupported constraints.

  For example, one could create an InverseKinematics problem with rich
  kinematic constraints, and then pass `InverseKinematics::prog()` into this
  option.
  */
  const solvers::MathematicalProgram* prog_with_additional_constraints{};

  /** For each constraint in `prog_with_additional_constraints`, IRIS will
  search for a counter-example by formulating a (likely nonconvex) optimization
  problem. The initial guess for this optimization is taken by sampling
  uniformly inside the current IRIS region. This option controls the
  termination condition for that counter-example search, defining the number of
  consecutive failures to find a counter-example requested before moving on to
  the next constraint. */
  int num_additional_constraint_infeasible_samples{5};

  /** The only randomization in IRIS is the random sampling done to find
  counter-examples for the additional constraints using in
  IrisInConfigurationSpace. Use this option to set the initial seed. */
  int random_seed{1234};
};

/** Configuration options for the IRIS algorithm running in rational
configuration space.

@ingroup geometry_optimization
*/
struct IrisOptionsRationalSpace : public IrisOptions {
  IrisOptionsRationalSpace() = default;

  // TODO(Alex.Amice) support turning this option on.
  /** For IRIS in rational configuration space, we can certify that the regions
   * are truly collision free using SOS programming and the methods in
   * multibody/rational_forward_kinematics. Turning this option on
   * certifies the regions everytime a set of hyperplanes is added */
  bool certify_region_with_sos_during_generation = false;

  // TODO(Alex.Amice) support turning this option on.
  /** For IRIS in rational configuration space, we can certify that the regions
   * are truly collision free using SOS programming and the methods in
   * multibody/rational_forward_kinematics. Turning this option on
   * certifies the regions at the end of the loop */
  bool certify_region_with_sos_after_generation = false;

  /** For IRIS in rational configuration space we need a point around which to
   * perform the stereographic projection. We will default to the zero vector if
   * not provided.
   * */
  std::optional<Eigen::VectorXd> q_star;
};


// takes t, p_AA, and p_BB and enforces that p_WA == p_WB
// class SamePointConstraintRational : public SamePointConstraint {
// public:
//  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SamePointConstraintRational)
//
//  SamePointConstraintRational(const multibody::RationalForwardKinematics*
//                                  rational_forward_kinematics_ptr,
//                              const Eigen::Ref<const Eigen::VectorXd>& q_star,
//                              const Context<double>& context)
//      : SamePointConstraint(&rational_forward_kinematics_ptr->plant(),
//      context),
//        rational_forward_kinematics_ptr_(rational_forward_kinematics_ptr),
//        q_star_(q_star) {}
//
//  ~SamePointConstraintRational() override {}
//
// private:
//  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
//              Eigen::VectorXd* y) const override;
//
//  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
//              AutoDiffVecXd* y) const override;
//
//  void DoEval(const Ref<const VectorX<symbolic::Variable>>& x,
//              VectorX<symbolic::Expression>* y) const override;
//
// protected:
//  const multibody::RationalForwardKinematics*
//  rational_forward_kinematics_ptr_; const Eigen::VectorXd q_star_;
//};

/** The IRIS (Iterative Region Inflation by Semidefinite programming) algorithm,
as described in

R. L. H. Deits and R. Tedrake, “Computing large convex regions of obstacle-free
space through semidefinite programming,” Workshop on the Algorithmic
Fundamentals of Robotics, Istanbul, Aug. 2014.
http://groups.csail.mit.edu/robotics-center/public_papers/Deits14.pdf

This algorithm attempts to locally maximize the volume of a convex polytope
representing obstacle-free space given a sample point and list of convex
obstacles. Rather than compute the volume of the polytope directly, the
algorithm maximizes the volume of an inscribed ellipsoid. It alternates between
finding separating hyperplanes between the ellipsoid and the obstacles and then
finding a new maximum-volume inscribed ellipsoid.

@param obstacles is a vector of convex sets representing the occupied space.
@param sample provides a point in the space; the algorithm is initialized using
a tiny sphere around this point. The algorithm is only guaranteed to succeed if
this sample point is collision free (outside of all obstacles), but in practice
the algorithm can often escape bad initialization (assuming the
require_sample_point_is_contained option is false).
@param domain describes the total region of interest; computed IRIS regions will
be inside this domain.  It must be bounded, and is typically a simple bounding
box (e.g. from HPolyhedron::MakeBox).

The @p obstacles, @p sample, and the @p domain must describe elements in the
same ambient dimension (but that dimension can be any positive integer).

@ingroup geometry_optimization
*/
HPolyhedron Iris(const ConvexSets& obstacles,
                 const Eigen::Ref<const Eigen::VectorXd>& sample,
                 const HPolyhedron& domain,
                 const IrisOptions& options = IrisOptions());

/** Constructs ConvexSet representations of obstacles for IRIS in 3D using the
geometry from a SceneGraph QueryObject. All geometry in the scene with a
proximity role, both anchored and dynamic, are consider to be *fixed*
obstacles frozen in the poses captured in the context used to create the
QueryObject.

When multiple representations are available for a particular geometry (e.g. a
Box can be represented as either an HPolyhedron or a VPolytope), then this
method will prioritize the representation that we expect is most performant
for the current implementation of the IRIS algorithm.

@ingroup geometry_optimization
*/
ConvexSets MakeIrisObstacles(
    const QueryObject<double>& query_object,
    std::optional<FrameId> reference_frame = std::nullopt);

/** A variation of the Iris (Iterative Region Inflation by Semidefinite
programming) algorithm which finds collision-free regions in the *configuration
space* of @p plant.  @see Iris for details on the original algorithm. This
variant uses nonlinear optimization (instead of convex optimization) to find
collisions in configuration space; each potential collision is
probabilistically "certified" by restarting the nonlinear optimization from
random initial seeds inside the candidate IRIS region until it fails to find a
collision in `options.num_collision_infeasible_samples` consecutive attempts.

@param plant describes the kinematics of configuration space.  It must be
connected to a SceneGraph in a systems::Diagram.
@param context is a context of the @p plant. The context must have the
positions of the plant set to the initialIRIS seed configuration.
@param options provides additional configuration options.  In particular,
increasing `options.num_collision_infeasible_samples` increases the chances that
the IRIS regions are collision free but can also significantly increase the
run-time of the algorithm. The same goes for
`options.num_additional_constraints_infeasible_samples`.

@throws std::exception if the sample configuration in @p context is infeasible.
@ingroup geometry_optimization
*/
HPolyhedron IrisInConfigurationSpace(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const IrisOptions& options = IrisOptions());

/** A variation of the Iris (Iterative Region Inflation by Semidefinite
programming) algorithm which finds collision-free regions in the *rational
parametrization of the configuration space* of @p plant. @see Iris for details
on the original algorithm. This is a reimplementation of
IrisInConfigurationSpace for the rational reparametrization

@param plant describes the kinematics of configuration space.  It must be
connected to a SceneGraph in a systems::Diagram.
@param context is a context of the @p plant. The context must have the positions
of the plant set to the initial IRIS seed configuration.
@param options provides additional configuration options.  In particular,
`options.certify_region_during_generation` vs
`options.certify_region_after_generation' can have an impact on computation time
@param starting_hpolyhedron is an optional argument to constrain the initial
Iris search. This defaults to the joint limits of the plants, but if there is a
reason to constrain it further this option is provided.
@ingroup geometry_optimization
*/
// HPolyhedron IrisInRationalConfigurationSpace(
//    const multibody::MultibodyPlant<double>& plant,
//    const systems::Context<double>& context,
//    const IrisOptionsRationalSpace& options = IrisOptionsRationalSpace(),
//    const std::optional<HPolyhedron>& starting_hpolyhedron = std::nullopt);

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
