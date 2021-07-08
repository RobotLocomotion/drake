#pragma once

#include <map>
#include <unordered_set>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mixed_integer_rotation_constraint.h"

namespace drake {
namespace multibody {
/** Solves the inverse kinematics problem as a mixed integer convex optimization
 * problem.
 * We use a mixed-integer convex relaxation of the rotation matrix. So if this
 * global inverse kinematics problem says the solution is infeasible, then it is
 * guaranteed that the kinematics constraints are not satisfiable.
 * If the global inverse kinematics returns a solution, the posture should
 * approximately satisfy the kinematics constraints, with some error.
 * The approach is described in Global Inverse Kinematics via Mixed-integer
 * Convex Optimization by Hongkai Dai, Gregory Izatt and Russ Tedrake,
 * International Journal of Robotics Research, 2019.
 *
 * @ingroup planning
 */
class GlobalInverseKinematics {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GlobalInverseKinematics)

  struct Options {
    // This constructor is needed, otherwise the compiler complains.
    Options() {}

    int num_intervals_per_half_axis{2};
    solvers::MixedIntegerRotationConstraintGenerator::Approach approach{
        solvers::MixedIntegerRotationConstraintGenerator::Approach::
            kBilinearMcCormick};
    solvers::IntervalBinning interval_binning{
        solvers::IntervalBinning::kLogarithmic};
    /** If true, add only mixed-integer linear constraints in the
     * constructor of GlobalInverseKinematics. The mixed-integer relaxation
     * is tighter with nonlinear constraints (such as Lorentz cone constraint)
     * than with linear constraints, but the optimization takes more time with
     * nonlinear constraints.
     */
    bool linear_constraint_only{false};
  };

  /**
   * Parses the robot kinematics tree. The decision variables include the
   * pose for each body (position/orientation). This constructor loops through
   * each body inside the robot kinematics tree, adds the constraint on each
   * body pose, so that the adjacent bodies are connected correctly by the joint
   * in between the bodies.
   * @param plant The robot on which the inverse kinematics problem is solved.
   * plant must be alive for as long as this object is around.
   * @param options The options to relax SO(3) constraint as mixed-integer
   * convex constraints. Refer to MixedIntegerRotationConstraintGenerator for
   * more details on the parameters in options.
   */
  explicit GlobalInverseKinematics(const MultibodyPlant<double>& plant,
                                   const Options& options = Options());

  ~GlobalInverseKinematics() {}

  const solvers::MathematicalProgram& prog() const { return prog_; }

  solvers::MathematicalProgram* get_mutable_prog() { return &prog_; }

  /** Getter for the decision variables on the rotation matrix `R_WB` for a body
   * with the specified index. This is the orientation of body i's frame
   * measured and expressed in the world frame.
   * @param body_index  The index of the queried body. Notice that body 0 is
   * the world, and thus not a decision variable.
   * @throws std::exception if the index is smaller than 1, or greater
   * than or equal to the total number of bodies in the robot.
   */
  const solvers::MatrixDecisionVariable<3, 3>& body_rotation_matrix(
      BodyIndex body_index) const;

  /** Getter for the decision variables on the position p_WBo of the body B's
   * origin measured and expressed in the world frame.
   * @param body_index The index of the queried body. Notice that body 0 is
   * the world, and thus not a decision variable.
   * @throws std::exception if the index is smaller than 1, or greater than
   * or equal to the total number of bodies in the robot.
   */
  const solvers::VectorDecisionVariable<3>& body_position(
      BodyIndex body_index) const;

  /**
   * After solving the inverse kinematics problem and finding out the pose of
   * each body, reconstruct the robot generalized position (joint angles, etc)
   * that matches with the body poses. Notice that since the rotation matrix is
   * approximated, that the solution of body_rotmat() might not be on SO(3)
   * exactly, the reconstructed body posture might not match with the body poses
   * exactly, and the kinematics constraint might not be satisfied exactly with
   * this reconstructed posture.
   * @warning Do not call this method if the problem is not solved successfully!
   * The returned value can be NaN or meaningless number if the problem is
   * not solved.
   * @retval q The reconstructed posture of the robot of the generalized
   * coordinates, corresponding to the RigidBodyTree on which the inverse
   * kinematics problem is solved.
   */
  Eigen::VectorXd ReconstructGeneralizedPositionSolution(
      const solvers::MathematicalProgramResult& result) const;

  /**
   * Adds the constraint that the position of a point `Q` on a body `B`
   * (whose index is `body_idx`), is within a box in a specified frame `F`.
   * The constraint is that the point `Q`'s position should lie within a
   * bounding box in the frame `F`. Namely
   *
   *    box_lb_F <= p_FQ <= box_ub_F
   *
   * where p_FQ is the position of the point Q measured and expressed in the
   * `F`, computed as
   *
   *    p_FQ = X_FW * (p_WBo + R_WB * p_BQ)
   *
   * hence this is a linear constraint on the decision variables p_WBo and R_WB.
   * The inequality is imposed elementwise.
   *
   * @note since the rotation matrix `R_WB` does not lie exactly on the
   * SO(3), due to the McCormick envelope relaxation, this constraint is subject
   * to the accumulated error from the root of the kinematics tree.
   *
   * @param body_idx The index of the body on which the position of a point is
   * constrained.
   * @param p_BQ The position of the point Q measured and expressed in the
   * body frame B.
   * @param box_lb_F The lower bound of the box in frame `F`.
   * @param box_ub_F The upper bound of the box in frame `F`.
   * @param X_WF. The frame in which the box is specified. This
   * frame is represented by an isometry transform X_WF, the transform from
   * the constraint frame F to the world frame W. Namely if the position of
   * the point `Q` in the world frame is `p_WQ`, then the constraint is
   *
   *     box_lb_F <= R_FW * (p_WQ-p_WFo) <= box_ub_F
   * where
   *   - R_FW is the rotation matrix of frame `W` expressed and measured in
   *     frame `F`. `R_FW = X_WF.linear().transpose()`.
   *   - p_WFo is the position of frame `F`'s origin, expressed and measured in
   *     frame `W`. `p_WFo = X_WF.translation()`.
   * @default is the identity transform.
   * @retval binding The newly added constraint, together with the bound
   * variables.
   */
  solvers::Binding<solvers::LinearConstraint> AddWorldPositionConstraint(
      BodyIndex body_idx, const Eigen::Vector3d& p_BQ,
      const Eigen::Vector3d& box_lb_F, const Eigen::Vector3d& box_ub_F,
      const Eigen::Isometry3d& X_WF = Eigen::Isometry3d::Identity());

  /**
   * Adds a constraint that the angle between the body orientation and the
   * desired orientation should not be larger than `angle_tol`. If we denote the
   * angle between two rotation matrices `R1` and `R2` as `θ`, i.e. θ is the
   * angle of the angle-axis representation of the rotation matrix `R1ᵀ * R2`,
   * we then know
   *
   *     trace(R1ᵀ * R2) = 2 * cos(θ) + 1
   * as in
   * http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
   * To constraint `θ < angle_tol`, we can impose the following constraint
   *
   *     2 * cos(angle_tol) + 1 <= trace(R1ᵀ * R2) <= 3
   *
   * @param body_idx The index of the body whose orientation will be
   * constrained.
   * @param desired_orientation The desired orientation of the body.
   * @param angle_tol The tolerance on the angle between the body orientation
   * and the desired orientation. Unit is radians.
   * @retval binding The newly added constraint, together with the bound
   * variables.
   */
  solvers::Binding<solvers::LinearConstraint> AddWorldOrientationConstraint(
      BodyIndex body_idx, const Eigen::Quaterniond& desired_orientation,
      double angle_tol);

  /** Penalizes the deviation to the desired posture.
   * For each body (except the world) in the kinematic tree, we add the cost
   *
   *     ∑ᵢ body_position_cost(i) * body_position_error(i) +
   *     body_orientation_cost(i) * body_orientation_error(i)
   * where `body_position_error(i)` is computed as the Euclidean distance error
   * |p_WBo(i) - p_WBo_desired(i)|
   * where
   * - p_WBo(i)        : position of body i'th origin `Bo` in the world frame
   *                     `W`.
   * - p_WBo_desired(i): position of body i'th origin `Bo` in the world frame
   *                     `W`, computed from the desired posture `q_desired`.
   *
   * body_orientation_error(i) is computed as (1 - cos(θ)), where θ is the
   * angle between the orientation of body i'th frame and body i'th frame using
   * the desired posture. Notice that 1 - cos(θ) = θ²/2 + O(θ⁴), so this cost
   * is on the square of θ, when θ is small.
   * Notice that since body 0 is the world, the cost on that body is always 0,
   * no matter what value `body_position_cost(0)` and `body_orientation_cost(0)`
   * take.
   * @param q_desired  The desired posture.
   * @param body_position_cost  The cost for each body's position error. Unit is
   * [1/m] (one over meters).
   * @pre
   * 1. body_position_cost.rows() == plant.num_bodies(), where `plant`
   *    is the input argument in the constructor of the class.
   * 2. body_position_cost(i) is non-negative.
   * @throws std::exception if the precondition is not satisfied.
   * @param body_orientation_cost The cost for each body's orientation error.
   * @pre
   * 1. body_orientation_cost.rows() == plant.num_bodies() , where
   *    `plant` is the input argument in the constructor of the class.
   * 2. body_position_cost(i) is non-negative.
   * @throws std::exception if the precondition is not satisfied.
   */
  void AddPostureCost(
      const Eigen::Ref<const Eigen::VectorXd>& q_desired,
      const Eigen::Ref<const Eigen::VectorXd>& body_position_cost,
      const Eigen::Ref<const Eigen::VectorXd>& body_orientation_cost);

  /**
   * Constrain the point `Q` lying within one of the convex polytopes.
   * Each convex polytope Pᵢ is represented by its vertices as
   * Pᵢ = ConvexHull(v_i1, v_i2, ... v_in). Mathematically we want to impose the
   * constraint that the p_WQ, i.e., the position of point `Q` in world frame
   * `W`, satisfies
   *
   *     p_WQ ∈ Pᵢ for one i.
   * To impose this constraint, we consider to introduce binary variable zᵢ, and
   * continuous variables w_i1, w_i2, ..., w_in for each vertex of Pᵢ, with the
   * following constraints
   *
   *     p_WQ = sum_i (w_i1 * v_i1 + w_i2 * v_i2 + ... + w_in * v_in)
   *     w_ij >= 0, ∀i,j
   *     w_i1 + w_i2 + ... + w_in = zᵢ
   *     sum_i zᵢ = 1
   *     zᵢ ∈ {0, 1}
   * Notice that if zᵢ = 0, then w_i1 * v_i1 + w_i2 * v_i2 + ... + w_in * v_in
   * is just 0.
   * This function can be used for collision avoidance, where each region Pᵢ is
   * a free space region. It can also be used for grasping, where each region Pᵢ
   * is a surface patch on the grasped object.
   * Note this approach also works if the region Pᵢ overlaps with each other.
   * @param body_index The index of the body to on which point `Q` is attached.
   * @param p_BQ The position of point `Q` in the body frame `B`.
   * @param region_vertices region_vertices[i] is the vertices for the i'th
   * region.
   * @retval z The newly added binary variables. If point `Q` is in the i'th
   * region, z(i) = 1.
   * @pre region_vertices[i] has at least 3 columns. Throw a std::runtime_error
   * if the precondition is not satisfied.
   */
  solvers::VectorXDecisionVariable BodyPointInOneOfRegions(
      BodyIndex body_index, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
      const std::vector<Eigen::Matrix3Xd>& region_vertices);

  /**
   * Describes a polytope in 3D as 𝐀 * 𝐱 ≤ 𝐛  (a set of half-spaces),
   * where 𝐀 ∈ ℝⁿˣ³, 𝐱 ∈ ℝ³, 𝐛 ∈ ℝⁿ.
   */
  struct Polytope3D {
    Polytope3D(const Eigen::Ref<const Eigen::MatrixX3d>& m_A,
               const Eigen::Ref<const Eigen::VectorXd>& m_b)
        : A{m_A}, b{m_b} {}
    Eigen::MatrixX3d A;
    Eigen::VectorXd b;
  };

  /**
   * Adds the constraint that a sphere rigidly attached to a body has to be
   * within at least one of the given bounded polytopes. If the polytopes don't
   * intersect, then the sphere is in one and only one polytope. Otherwise the
   * sphere is in at least one of the polytopes (could be in the intersection of
   * multiple polytopes.)
   * If the i'th polytope is described as
   *
   *     Aᵢ * x ≤ bᵢ
   * where Aᵢ ∈ ℝⁿ ˣ ³, bᵢ ∈ ℝⁿ.
   * Then a sphere with center position p_WQ and radius r is within the i'th
   * polytope, if
   *
   *     Aᵢ * p_WQ ≤ bᵢ - aᵢr
   * where aᵢ(j) = Aᵢ.row(j).norm()
   * To constrain that the sphere is in one of the n polytopes, we introduce the
   * binary variable z ∈{0, 1}ⁿ, together with continuous variables yᵢ ∈ ℝ³, i
   * = 1, ..., n, such that
   * p_WQ = y₁ + ... + yₙ
   * Aᵢ * yᵢ ≤ (bᵢ - aᵢr)zᵢ
   * z₁ + ... +zₙ  = 1
   * Notice that when zᵢ = 0, Aᵢ * yᵢ ≤ 0 implies that yᵢ = 0. This is due to
   * the boundedness of the polytope. If Aᵢ * yᵢ ≤ 0 has a non-zero solution y̅,
   * that y̅ ≠ 0 and Aᵢ * y̅ ≤ 0. Then for any point x̂ in the polytope satisfying
   * Aᵢ * x̂ ≤ bᵢ, we know the ray x̂ + ty̅, ∀ t ≥ 0 also satisfies Aᵢ * (x̂ + ty̅)
   * ≤ bᵢ, thus the ray is within the polytope, violating the boundedness
   * assumption.
   * @param body_index The index of the body to which the sphere is attached.
   * @param p_BQ The position of the sphere center in the body frame B.
   * @param radius The radius of the sphere.
   * @param polytopes. polytopes[i] = (Aᵢ, bᵢ). We assume that Aᵢx≤ bᵢ is a
   * bounded polytope. It is the user's responsibility to guarantee the
   * boundedness.
   * @retval z The newly added binary variables. If z(i) = 1, then the sphere is
   * in the i'th polytope. If two or more polytopes are intersecting, and the
   * sphere is in the intersection region, then it is up to the solver to choose
   * one of z(i) to be 1.
   */
  solvers::VectorXDecisionVariable BodySphereInOneOfPolytopes(
      BodyIndex body_index, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
      double radius, const std::vector<Polytope3D>& polytopes);

  /**
   * Adds joint limits on a specified joint.
   * @param body_index The joint connecting the parent link to this body will be
   * constrained.
   * @param joint_lower_bound The lower bound for the joint.
   * @param joint_upper_bound The upper bound for the joint.
   * @param linear_constraint_approximation If true, joint limits are
   * approximated as linear constraints on parent and child link orientations,
   * otherwise they are imposed as Lorentz cone constraints.
   * With the Lorentz cone formulation, the joint limit constraint would be
   * tight if our mixed-integer constraint on SO(3) were tight. By enforcing the
   * joint limits as linear constraint, the original inverse kinematics problem
   * is further relaxed, on top of SO(3) relaxation, but potentially with faster
   * computation. @default is false.
   */
  void AddJointLimitConstraint(BodyIndex body_index, double joint_lower_bound,
                               double joint_upper_bound,
                               bool linear_constraint_approximation = false);

 private:
  // This is an utility function for `ReconstructGeneralizedPositionSolution`.
  // This function computes the joint generalized position on the body with
  // index body_idx. Note that the orientation of the parent link of the body
  // body_idx should have been reconstructed, in reconstruct_R_WB.
  void ReconstructGeneralizedPositionSolutionForBody(
      const solvers::MathematicalProgramResult& result, BodyIndex body_idx,
      const std::map<BodyIndex, JointIndex>& body_to_joint_map,
      const std::unordered_set<BodyIndex>& weld_to_world_body_index_set,
      Eigen::Ref<Eigen::VectorXd> q,
      std::vector<Eigen::Matrix3d>* reconstruct_R_WB) const;

  solvers::MathematicalProgram prog_;

  const MultibodyPlant<double>& plant_;

  // joint_lower_bounds_ and joint_upper_bounds_ are column vectors of size
  // plant->get_num_positions() x 1.
  // joint_lower_bounds_(i) is the lower bound of the i'th joint.
  // joint_upper_bounds_(i) is the upper bound of the i'th joint.
  // These joint bounds include those specified in the plant (like in the URDF
  // file), and the bounds imposed by the user, through AddJointLimitConstraint.
  Eigen::VectorXd joint_lower_bounds_;
  Eigen::VectorXd joint_upper_bounds_;

  // R_WB_[i] is the orientation of body i in the world reference frame,
  // it is expressed in the world frame.
  std::vector<solvers::MatrixDecisionVariable<3, 3>> R_WB_;

  // p_WBo_[i] is the position of the origin Bo of body frame B for the i'th
  // body, measured and expressed in the world frame.
  std::vector<solvers::VectorDecisionVariable<3>> p_WBo_;
};
}  // namespace multibody
}  // namespace drake
