#pragma once

#include <memory>
#include <optional>

#include "drake/common/sorted_pair.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
/**
 * Solves an inverse kinematics (IK) problem on a MultibodyPlant, to find the
 * postures of the robot satisfying certain constraints.
 * The decision variables include the generalized position of the robot.
 *
 * To perform IK on a subset of the plant, use the constructor overload that
 * takes a `plant_context` and use `Joint::Lock` on the joints in that Context
 * that should be fixed during IK.
 *
 * @ingroup planning_kinematics
 */
class InverseKinematics {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseKinematics);

  ~InverseKinematics() {}

  /**
   * Constructs an inverse kinematics problem for a MultibodyPlant.
   * This constructor will create and own a context for `plant`.
   * @param plant The robot on which the inverse kinematics problem will be
   * solved.
   * @param with_joint_limits If set to true, then the constructor
   * imposes the joint limits (obtained from plant.GetPositionLowerLimits()
   * and plant.GetPositionUpperLimits()). If set to false, then the constructor
   * does not impose the joint limit constraints in the constructor.
   * @note The inverse kinematics problem constructed in this way doesn't permit
   * collision related constraint (such as calling
   * AddMinimumDistanceConstraint). To enable collision related constraint, call
   * InverseKinematics(const MultibodyPlant<double>& plant,
   * systems::Context<double>* plant_context);
   */
  explicit InverseKinematics(const MultibodyPlant<double>& plant,
                             bool with_joint_limits = true);

  /**
   * Constructs an inverse kinematics problem for a MultibodyPlant. If the user
   * wants to solve the problem with collision related constraint (like calling
   * AddMinimumDistanceConstraint), please use this constructor.
   * @param plant The robot on which the inverse kinematics problem will be
   * solved. This plant should have been connected to a SceneGraph within a
   * Diagram
   * @param plant_context The context for the plant. This context should be a
   * part of the Diagram context. Any locked joints in the `plant_context` will
   * remain fixed at their locked value. (This provides a convenient way to
   * perform IK on a subset of the plant.) To construct a plant connected to a
   * SceneGraph, with the corresponding plant_context, the steps are:
   * ```
   * // 1. Add a diagram containing the MultibodyPlant and SceneGraph
   * systems::DiagramBuilder<double> builder;
   * auto items = AddMultibodyPlantSceneGraph(&builder, 0.0);
   * // 2. Add collision geometries to the plant
   * Parser(&builder).AddModels("model.sdf");
   * // 3. Construct the diagram
   * auto diagram = builder.Build();
   * // 4. Create diagram context.
   * auto diagram_context= diagram->CreateDefaultContext();
   * // 5. Get the context for the plant.
   * auto plant_context = &(diagram->GetMutableSubsystemContext(items.plant,
   * diagram_context.get()));
   * ```
   * This context will be modified during calling ik.prog.Solve(...). When
   * Solve() returns `result`, context will store the optimized posture, namely
   * plant.GetPositions(*context) will be the same as in
   * result.GetSolution(ik.q()). The user could then use this context to perform
   * kinematic computation (like computing the position of the end-effector
   * etc.).
   * @param with_joint_limits If set to true, then the constructor imposes the
   * joint limits (obtained from plant.GetPositionLowerLimits() and
   * plant.GetPositionUpperLimits(), and from any body/joint locks set in
   * `plant_context`). If set to false, then the constructor does not impose
   * the joint limit constraints in the constructor.  */
  InverseKinematics(const MultibodyPlant<double>& plant,
                    systems::Context<double>* plant_context,
                    bool with_joint_limits = true);

  /** Adds the kinematic constraint that a point Q, fixed in frame B, should lie
   * within a bounding box expressed in another frame A as p_AQ_lower <= p_AQ <=
   * p_AQ_upper, where p_AQ is the position of point Q measured and expressed
   * in frame A.
   * @param frameB The frame in which point Q is fixed.
   * @param p_BQ The position of the point Q, rigidly attached to frame B,
   * measured and expressed in frame B.
   * @param frameA The frame in which the bounding box p_AQ_lower <= p_AQ <=
   * p_AQ_upper is expressed.
   * @param p_AQ_lower The lower bound on the position of point Q, measured and
   * expressed in frame A.
   * @param p_AQ_upper The upper bound on the position of point Q, measured and
   * expressed in frame A.
   */
  solvers::Binding<solvers::Constraint> AddPositionConstraint(
      const Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
      const Frame<double>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper);

  /** Adds the kinematic constraint that a point Q, fixed in frame B, should lie
   * within a bounding box expressed in another frame A as p_AQ_lower <= p_AQ <=
   * p_AQ_upper, where p_AQ is the position of point Q measured and expressed
   * in frame A.
   * @param frameB The frame in which point Q is fixed.
   * @param p_BQ The position of the point Q, rigidly attached to frame B,
   * measured and expressed in frame B.
   * @param frameAbar We will compute frame A from frame Abar. The bounding box
   * p_AQ_lower <= p_AQ <= p_AQ_upper is expressed in frame A.
   * @param X_AbarA The relative transform between frame Abar and A. If empty,
   * then we use the identity transform.
   * @param p_AQ_lower The lower bound on the position of point Q, measured and
   * expressed in frame A.
   * @param p_AQ_upper The upper bound on the position of point Q, measured and
   * expressed in frame A.
   */
  solvers::Binding<solvers::Constraint> AddPositionConstraint(
      const Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
      const Frame<double>& frameAbar,
      const std::optional<math::RigidTransformd>& X_AbarA,
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper);

  /** Adds a cost of the form (p_AP - p_AQ)ᵀ C (p_AP - p_AQ), where point P is
   * specified relative to frame A and point Q is specified relative to frame
   * B, and the cost is evaluated in frame A.
   * @param frameA The frame in which point P's position is measured.
   * @param p_AP The point P.
   * @param frameB The frame in which point Q's position is measured.
   * @param p_BQ The point Q.
   * @param C A 3x3 matrix representing the cost in quadratic form.
   */
  solvers::Binding<solvers::Cost> AddPositionCost(
      const Frame<double>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& p_AP,
      const Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
      const Eigen::Ref<const Eigen::Matrix3d>& C);

  /**
   * Constrains that the angle difference θ between the orientation of frame A
   * and the orientation of frame B to satisfy θ ≤ θ_bound. Frame A is fixed to
   * frame A_bar, with orientation R_AbarA measured in frame A_bar. Frame B is
   * fixed to frame B_bar, with orientation R_BbarB measured in frame B_bar. The
   * angle difference between frame A's orientation R_WA and B's orientation
   * R_WB is θ, (θ ∈ [0, π]), if there exists a rotation axis a, such that
   * rotating frame A by angle θ about axis a aligns it with frame B. Namely
   * R_AB = I + sinθ â + (1-cosθ)â²   (1)
   * where R_AB is the orientation of frame B expressed in frame A. â is the
   * skew symmetric matrix of the rotation axis a. Equation (1) is the Rodrigues
   * formula that computes the rotation matrix from a rotation axis a and an
   * angle θ, https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
   * If the users want frame A and frame B to align perfectly, they can set
   * θ_bound = 0.
   * Mathematically, this constraint is imposed as
   * trace(R_AB) ≥ 2cos(θ_bound) + 1   (1)
   * To derive (1), using Rodrigues formula
   * R_AB = I + sinθ â + (1-cosθ)â²
   * where
   * trace(R_AB) = 2cos(θ) + 1 ≥ 2cos(θ_bound) + 1
   * @param frameAbar frame A_bar, the frame A is fixed to frame A_bar.
   * @param R_AbarA The orientation of frame A measured in frame A_bar.
   * @param frameBbar frame B_bar, the frame B is fixed to frame B_bar.
   * @param R_BbarB The orientation of frame B measured in frame B_bar.
   * @param theta_bound The bound on the angle difference between frame A's
   * orientation and frame B's orientation. It is denoted as θ_bound in the
   * documentation. @p theta_bound is in radians.
   */
  solvers::Binding<solvers::Constraint> AddOrientationConstraint(
      const Frame<double>& frameAbar,
      const math::RotationMatrix<double>& R_AbarA,
      const Frame<double>& frameBbar,
      const math::RotationMatrix<double>& R_BbarB, double theta_bound);

  /** Adds a cost of the form `c * (1 - cos(θ))`, where θ is the angle between
   * the orientation of frame A and the orientation of frame B, and @p c is a
   * cost scaling.
   * @param frameAbar A frame on the MultibodyPlant.
   * @param R_AbarA The rotation matrix describing the orientation of frame A
   * relative to Abar.
   * @param frameBbar A frame on the MultibodyPlant.
   * @param R_BbarB The rotation matrix describing the orientation of frame B
   * relative to Bbar.
   * @param c A scalar cost weight.
   */
  solvers::Binding<solvers::Cost> AddOrientationCost(
      const Frame<double>& frameAbar,
      const math::RotationMatrix<double>& R_AbarA,
      const Frame<double>& frameBbar,
      const math::RotationMatrix<double>& R_BbarB, double c);

  /**
   * Constrains a target point T to be within a cone K. The point T ("T" stands
   * for "target") is fixed in a frame B, with position p_BT. The cone
   * originates from a point S ("S" stands for "source"), fixed in frame A with
   * position p_AS, with the axis of the cone being n, also fixed
   * in frame A. The half angle of the cone is θ. A common usage of this
   * constraint is that a camera should gaze at some target; namely the target
   * falls within a gaze cone, originating from the camera eye.
   * @param frameA The frame where the gaze cone is fixed to.
   * @param p_AS The position of the cone source point S, measured and
   * expressed in frame A.
   * @param n_A The directional vector representing the center ray of the
   * cone, expressed in frame A.
   * @pre @p n_A cannot be a zero vector.
   * @throws std::exception is n_A is close to a zero vector.
   * @param frameB The frame where the target point T is fixed to.
   * @param p_BT The position of the target point T, measured and expressed in
   * frame B.
   * @param cone_half_angle The half angle of the cone. We denote it as θ in the
   * documentation. @p cone_half_angle is in radians.
   * @pre @p 0 <= cone_half_angle <= pi.
   * @throws std::exception if cone_half_angle is outside of the bound.
   */
  solvers::Binding<solvers::Constraint> AddGazeTargetConstraint(
      const Frame<double>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& p_AS,
      const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle);

  /**
   * Constrains that the angle between a vector na and another vector nb is
   * between [θ_lower, θ_upper]. na is fixed to a frame A, while nb is fixed
   * to a frame B.
   * Mathematically, if we denote na_unit_A as na expressed in frame A after
   * normalization (na_unit_A has unit length), and nb_unit_B as nb expressed in
   * frame B after normalization, the constraint is
   * cos(θ_upper) ≤ na_unit_Aᵀ * R_AB * nb_unit_B ≤ cos(θ_lower), where R_AB is
   * the rotation matrix, representing the orientation of frame B expressed in
   * frame A.
   * @param frameA The frame to which na is fixed.
   * @param na_A The vector na fixed to frame A, expressed in frame A.
   * @pre na_A should be a non-zero vector.
   * @throws std::exception if na_A is close to zero.
   * @param frameB The frame to which nb is fixed.
   * @param nb_B The vector nb fixed to frame B, expressed in frame B.
   * @pre nb_B should be a non-zero vector.
   * @throws std::exception if nb_B is close to zero.
   * @param angle_lower The lower bound on the angle between na and nb. It is
   * denoted as θ_lower in the documentation. @p angle_lower is in radians.
   * @pre angle_lower >= 0.
   * @throws std::exception if angle_lower is negative.
   * @param angle_upper The upper bound on the angle between na and nb. it is
   * denoted as θ_upper in the class documentation. @p angle_upper is in
   * radians.
   * @pre angle_lower <= angle_upper <= pi.
   * @throws std::exception if angle_upper is outside the bounds.
   */
  solvers::Binding<solvers::Constraint> AddAngleBetweenVectorsConstraint(
      const Frame<double>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& na_A,
      const Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& nb_B, double angle_lower,
      double angle_upper);

  /**
   * Add a cost c * (1-cosθ) where θ is the angle between the vector `na` and
   * `nb`. na is fixed to a frame A, while nb is fixed to a frame B.
   * @param frameA The frame to which na is fixed.
   * @param na_A The vector na fixed to frame A, expressed in frame A.
   * @pre na_A should be a non-zero vector.
   * @throws std::exception if na_A is close to zero.
   * @param frameB The frame to which nb is fixed.
   * @param nb_B The vector nb fixed to frame B, expressed in frame B.
   * @pre nb_B should be a non-zero vector.
   * @throws std::exception if nb_B is close to zero.
   * @param c The cost is c * (1-cosθ).
   */
  solvers::Binding<solvers::Cost> AddAngleBetweenVectorsCost(
      const Frame<double>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& na_A,
      const Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& nb_B, double c);

  /**
   * Adds the constraint that the pairwise distance between objects should be no
   * smaller than `bound`. We consider the distance between
   * pairs of
   * 1. Anchored (static) object and a dynamic object.
   * 2. A dynamic object and another dynamic object, if one is not the parent
   * link of the other.
   * @param bound The minimum allowed value, dₘᵢₙ, of the signed
   * distance between any candidate pair of geometries.
   * @param influence_distance_offset See MinimumDistanceLowerBoundConstraint
   * for explanation.
   * @pre The MultibodyPlant passed to the constructor of `this` has registered
   * its geometry with a SceneGraph.
   * @pre 0 < `influence_distance_offset` < ∞
   */
  solvers::Binding<solvers::Constraint> AddMinimumDistanceLowerBoundConstraint(
      double bound, double influence_distance_offset = 0.01);

  /**
   Adds the constraint that at least one pair of geometries has distance no
   larger than `bound`. We consider the distance between pairs
   of
   1. Anchored (static) object and a dynamic object.
   2. A dynamic object and another dynamic object, if one is not the parent
   link of the other.
   @param bound The upper bound of the minimum signed distance
   between any candidate pair of geometries. Notice this is NOT the upper bound
   of every distance, but the upper bound of the smallest distance.
   @param influence_distance_offset  See MinimumDistanceUpperBoundConstraint
   for more details on influence_distance_offset.
   @pre The MultibodyPlant passed to the constructor of `this` has registered
   its geometry with a SceneGraph.
   @pre 0 < `influence_distance_offset` < ∞
   */
  solvers::Binding<solvers::Constraint> AddMinimumDistanceUpperBoundConstraint(
      double bound, double influence_distance_offset);

  /**
   * Adds the constraint that the distance between a pair of geometries is
   * within some bounds.
   * @param geometry_pair The pair of geometries between which the distance is
   * constrained. Notice that we only consider the distance between a static
   * geometry and a dynamic geometry, or a pair of dynamic geometries. We don't
   * allow constraining the distance between two static geometries.
   * @param distance_lower The lower bound on the distance.
   * @param distance_upper The upper bound on the distance.
   */
  solvers::Binding<solvers::Constraint> AddDistanceConstraint(
      const SortedPair<geometry::GeometryId>& geometry_pair,
      double distance_lower, double distance_upper);

  /**
   * Add a constraint that the distance between point P1 attached to frame 1 and
   * point P2 attached to frame 2 is within the range [distance_lower,
   * distance_upper].
   * @param frame1 The frame to which P1 is attached.
   * @param p_B1P1 The position of P1 measured and expressed in frame 1.
   * @param frame2 The frame to which P2 is attached.
   * @param p_B2P2 The position of P2 measured and expressed in frame 2.
   * @param distance_lower The lower bound on the distance.
   * @param distance_upper The upper bound on the distance.
   */
  solvers::Binding<solvers::Constraint> AddPointToPointDistanceConstraint(
      const Frame<double>& frame1,
      const Eigen::Ref<const Eigen::Vector3d>& p_B1P1,
      const Frame<double>& frame2,
      const Eigen::Ref<const Eigen::Vector3d>& p_B2P2, double distance_lower,
      double distance_upper);

  /**
   * Add a constraint that the distance between point P attached to frame_point
   * (denoted as B1) and a line attached to frame_line (denoted as B2) is within
   * the range [distance_lower, distance_upper]. The line passes through a point
   * Q with a directional vector n.
   * @param frame_point The frame to which P is attached.
   * @param p_B1P The position of P measured and expressed in frame_point.
   * @param frame_line The frame to which the line is attached.
   * @param p_B2Q The position of Q measured and expressed in frame_line, the
   * line passes through Q.
   * @param n_B2 The direction vector of the line measured and expressed in
   * frame_line.
   * @param distance_lower The lower bound on the distance.
   * @param distance_upper The upper bound on the distance.
   */
  solvers::Binding<solvers::Constraint> AddPointToLineDistanceConstraint(
      const Frame<double>& frame_point,
      const Eigen::Ref<const Eigen::Vector3d>& p_B1P,
      const Frame<double>& frame_line,
      const Eigen::Ref<const Eigen::Vector3d>& p_B2Q,
      const Eigen::Ref<const Eigen::Vector3d>& n_B2, double distance_lower,
      double distance_upper);

  /**
   * Adds the constraint that the position of P1, ..., Pn satisfy A *
   * [p_FP1; p_FP2; ...; p_FPn] <= b.
   * @param frameF The frame in which the position P is measured and expressed
   * @param frameG The frame in which the point P is rigidly attached.
   * @param p_GP p_GP.col(i) is the position of the i'th point Pi measured and
   * expressed in frame G.
   * @param A We impose the constraint A * [p_FP1; p_FP2; ...; p_FPn] <= b. @pre
   * A.cols() = 3 * p_GP.cols().
   * @param b We impose the constraint A * [p_FP1; p_FP2; ...; p_FPn] <= b.
   */
  solvers::Binding<solvers::Constraint> AddPolyhedronConstraint(
      const Frame<double>& frameF, const Frame<double>& frameG,
      const Eigen::Ref<const Eigen::Matrix3Xd>& p_GP,
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b);

  /** Getter for q. q is the decision variable for the generalized positions of
   * the robot. */
  const solvers::VectorXDecisionVariable& q() const { return q_; }

  /** Getter for the optimization program constructed by InverseKinematics. */
  const solvers::MathematicalProgram& prog() const { return *prog_; }

  /** Getter for the optimization program constructed by InverseKinematics. */
  solvers::MathematicalProgram* get_mutable_prog() { return prog_.get(); }

  /** Getter for the plant context. */
  const systems::Context<double>& context() const { return *context_; }

  /** Getter for the mutable plant context. */
  systems::Context<double>* get_mutable_context() { return context_; }

 private:
  /* Both public constructors delegate to here. Exactly one of owned_context or
  plant_context must be non-null. */
  InverseKinematics(const MultibodyPlant<double>& plant,
                    std::unique_ptr<systems::Context<double>> owned_context,
                    systems::Context<double>* plant_context,
                    bool with_joint_limits);

  std::unique_ptr<solvers::MathematicalProgram> prog_;
  const MultibodyPlant<double>& plant_;
  std::unique_ptr<systems::Context<double>> const owned_context_;
  systems::Context<double>* const context_;
  solvers::VectorXDecisionVariable q_;
};
}  // namespace multibody
}  // namespace drake
