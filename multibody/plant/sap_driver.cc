#include "drake/multibody/plant/sap_driver.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_limit_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"

using drake::geometry::GeometryId;
using drake::math::RotationMatrix;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::ExtractNormal;
using drake::multibody::contact_solvers::internal::ExtractTangent;
using drake::multibody::contact_solvers::internal::MatrixBlock;
using drake::multibody::contact_solvers::internal::SapConstraint;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapFrictionConeConstraint;
using drake::multibody::contact_solvers::internal::SapHolonomicConstraint;
using drake::multibody::contact_solvers::internal::SapLimitConstraint;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
SapDriver<T>::SapDriver(const CompliantContactManager<T>* manager,
                        double near_rigid_parameter)
    : manager_(manager), near_rigid_parameter_(near_rigid_parameter) {
  // Collect joint damping coefficients into a vector.
  joint_damping_ = VectorX<T>::Zero(plant().num_velocities());
  for (JointIndex j(0); j < plant().num_joints(); ++j) {
    const Joint<T>& joint = plant().get_joint(j);
    const int velocity_start = joint.velocity_start();
    const int nv = joint.num_velocities();
    joint_damping_.segment(velocity_start, nv) = joint.damping_vector();
  }
}

template <typename T>
SapDriver<T>::SapDriver(const CompliantContactManager<T>* manager)
    : SapDriver(manager, 1.0) {}

template <typename T>
void SapDriver<T>::set_sap_solver_parameters(
    const contact_solvers::internal::SapSolverParameters& parameters) {
  sap_parameters_ = parameters;
}

template <typename T>
void SapDriver<T>::DeclareCacheEntries(
    CompliantContactManager<T>* mutable_manager) {
  DRAKE_DEMAND(mutable_manager == manager_);

  const systems::DependencyTicket xd_ticket = systems::System<T>::xd_ticket();
  const systems::DependencyTicket inputs_ticket =
      plant().all_input_ports_ticket();
  const systems::DependencyTicket parameters_ticket =
      plant().all_parameters_ticket();
  const std::set<systems::DependencyTicket> state_input_and_parameters = {
      xd_ticket, inputs_ticket, parameters_ticket};

  const auto& contact_problem_cache_entry = mutable_manager->DeclareCacheEntry(
      "contact problem",
      systems::ValueProducer(this, ContactProblemCache<T>(plant().time_step()),
                             &SapDriver<T>::CalcContactProblemCache),
      state_input_and_parameters);
  contact_problem_ = contact_problem_cache_entry.cache_index();
}

template <typename T>
const ContactProblemCache<T>& SapDriver<T>::EvalContactProblemCache(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(contact_problem_)
      .template Eval<ContactProblemCache<T>>(context);
}

template <typename T>
void SapDriver<T>::CalcLinearDynamicsMatrix(const systems::Context<T>& context,
                                            std::vector<MatrixX<T>>* A) const {
  DRAKE_DEMAND(A != nullptr);
  A->resize(tree_topology().num_trees());
  const int nv = plant().num_velocities();

  // TODO(amcastro-tri): consider placing the computation of the dense mass
  // matrix in a cache entry to minimize heap allocations or better yet,
  // implement a MultibodyPlant method to compute the per-tree mass matrices.
  MatrixX<T> M(nv, nv);
  plant().CalcMassMatrix(context, &M);

  // The driver solves free motion velocities using a discrete scheme with
  // implicit joint dissipation. That is, it solves the momentum balance:
  //   m(v) = (M + dt⋅D)⋅(v-v₀)/dt - k(x₀) = 0
  // where k(x₀) are all the non-constraint forces such as Coriolis terms and
  // external actuation, evaluated at the previous state x₀.
  // The dynamics matrix is defined as:
  //   A = ∂m/∂v = (M + dt⋅D)
  M.diagonal() += plant().time_step() * joint_damping_;

  for (TreeIndex t(0); t < tree_topology().num_trees(); ++t) {
    const int tree_start = tree_topology().tree_velocities_start(t);
    const int tree_nv = tree_topology().num_tree_velocities(t);
    (*A)[t] = M.block(tree_start, tree_start, tree_nv, tree_nv);
  }

  if constexpr (std::is_same_v<T, double>) {
    if (manager().deformable_driver_ != nullptr) {
      manager().deformable_driver_->AppendLinearDynamicsMatrix(context, A);
    }
  }
}

template <typename T>
void SapDriver<T>::CalcFreeMotionVelocities(const systems::Context<T>& context,
                                            VectorX<T>* v_star) const {
  DRAKE_DEMAND(v_star != nullptr);
  // N.B. Forces are evaluated at the previous time step state. This is
  // consistent with the explicit Euler and symplectic Euler schemes.
  // TODO(amcastro-tri): Implement free-motion velocities update based on the
  // theta-method, as in the SAP paper.
  const VectorX<T>& vdot0 =
      manager().EvalAccelerationsDueToNonContactForcesCache(context).get_vdot();
  const double dt = this->plant().time_step();
  const VectorX<T>& x0 =
      context.get_discrete_state(manager().multibody_state_index()).value();
  const auto v0 = x0.bottomRows(this->plant().num_velocities());
  if constexpr (std::is_same_v<T, double>) {
    if (manager().deformable_driver_ != nullptr) {
      const VectorX<T>& deformable_v_star =
          manager().deformable_driver_->EvalParticipatingFreeMotionVelocities(
              context);
      const int rigid_dofs = v0.size();
      const int deformable_dofs = deformable_v_star.size();
      v_star->resize(rigid_dofs + deformable_dofs);
      v_star->head(rigid_dofs) = v0 + dt * vdot0;
      v_star->tail(deformable_dofs) = deformable_v_star;
    } else {
      *v_star = v0 + dt * vdot0;
    }
  } else {
    *v_star = v0 + dt * vdot0;
  }
}

template <typename T>
std::vector<RotationMatrix<T>> SapDriver<T>::AddContactConstraints(
    const systems::Context<T>& context, SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  // Parameters used by SAP to estimate regularization, see [Castro et al.,
  // 2021].
  // TODO(amcastro-tri): consider exposing these parameters.
  constexpr double sigma = 1.0e-3;

  const std::vector<DiscreteContactPair<T>>& contact_pairs =
      manager().EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();

  // Quick no-op exit.
  if (num_contacts == 0) return std::vector<RotationMatrix<T>>();

  std::vector<ContactPairKinematics<T>> contact_kinematics =
      manager().CalcContactKinematics(context);

  std::vector<RotationMatrix<T>> R_WC;
  R_WC.reserve(num_contacts);
  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& discrete_pair = contact_pairs[icontact];

    const T stiffness = discrete_pair.stiffness;
    const T dissipation_time_scale = discrete_pair.dissipation_time_scale;
    const T friction = discrete_pair.friction_coefficient;
    const T phi = contact_kinematics[icontact].phi;
    const auto& jacobian_blocks = contact_kinematics[icontact].jacobian;

    const typename SapFrictionConeConstraint<T>::Parameters parameters{
        friction, stiffness, dissipation_time_scale, near_rigid_parameter_,
        sigma};

    if (jacobian_blocks.size() == 1) {
      problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<T>>(
          jacobian_blocks[0].tree, std::move(jacobian_blocks[0].J), phi,
          parameters));
    } else {
      problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<T>>(
          jacobian_blocks[0].tree, jacobian_blocks[1].tree,
          std::move(jacobian_blocks[0].J), std::move(jacobian_blocks[1].J), phi,
          parameters));
    }
    R_WC.emplace_back(std::move(contact_kinematics[icontact].R_WC));
  }
  return R_WC;
}

template <typename T>
void SapDriver<T>::AddLimitConstraints(const systems::Context<T>& context,
                                       const VectorX<T>& v_star,
                                       SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  constexpr double kInf = std::numeric_limits<double>::infinity();

  // TODO(amcastro-tri): consider exposing these parameters.
  // "Near-rigid" parameter. See [Castro et al., 2021].
  constexpr double kBeta = 0.1;
  // Parameter used to estimate the size of a window [w_l, w_u] within which we
  // expect the configuration q for a given joint to be in the next time step.
  // See notes below for details. Dimensionless.
  constexpr double kLimitWindowFactor = 2.0;

  const double dt = plant().time_step();

  // N.B. MultibodyPlant estimates very conservative (soft) stiffness and
  // damping parameters to ensure that the explicit treatment of the compliant
  // forces used to impose limits does not become unstable. SAP however treats
  // these forces implicitly and therefore these parameters can be tighten for
  // stiffer limits. Here we set the stiffness parameter to a very high value so
  // that SAP works in the "near-rigid" regime as described in the SAP paper,
  // [Castro et al., 2021]. As shown in the SAP paper, a dissipation timescale
  // of the order of the time step leads to a critically damped constraint.
  // N.B. Units of stiffness (say N/m for a translational q) are consistent
  // with the units of the corresponding generalized coordinate (say m for a
  // translational q) so that their product has units of the corresponding
  // generalized force (say N for a translational q).
  // TODO(amcastro-tri): allow users to specify joint limits stiffness and
  // damping.
  const double stiffness = 1.0e12;
  const double dissipation_time_scale = dt;

  for (JointIndex joint_index(0); joint_index < plant().num_joints();
       ++joint_index) {
    const Joint<T>& joint = plant().get_joint(joint_index);
    // We only support limits for 1 DOF joints for which we know that q̇ = v.
    if (joint.num_positions() == 1 && joint.num_velocities() == 1) {
      const double lower_limit = joint.position_lower_limits()[0];
      const double upper_limit = joint.position_upper_limits()[0];
      const int velocity_start = joint.velocity_start();
      const TreeIndex tree_index =
          tree_topology().velocity_to_tree_index(velocity_start);
      const int tree_nv = tree_topology().num_tree_velocities(tree_index);
      const int tree_velocity_start =
          tree_topology().tree_velocities_start(tree_index);
      const int tree_dof = velocity_start - tree_velocity_start;

      // Current configuration position.
      const T& q0 = joint.GetOnePosition(context);
      const T& v0 = joint.GetOneVelocity(context);

      // Estimate a window size around q0. In order to build a smaller
      // optimization problem, we only add a constraint if the joint
      // limits are within this window.
      using std::abs;
      using std::max;
      // delta_q estimates how much q changes in a single time step.
      // We use the maximum of v0 and v* for a conservative estimation.
      const T delta_q = dt * max(abs(v0), abs(v_star(velocity_start)));
      // We use a factor kLimitWindowFactor to look into a larger window. A very
      // large kLimitWindowFactor means that constraints will always be added
      // even if they are inactive at the end of the computation. A smaller
      // kLimitWindowFactor will result in a smaller problem, faster to solve,
      // though constraints could be missed until the next time step.
      const T window_lower = q0 - kLimitWindowFactor * delta_q;
      const T window_upper = q0 + kLimitWindowFactor * delta_q;

      // N.B. window_lower < window_upper by definition.
      const double ql = lower_limit < window_lower ? -kInf : lower_limit;
      const double qu = upper_limit > window_upper ? kInf : upper_limit;

      // Constraint is added only when one of ql and qu is finite.
      if (!std::isinf(ql) || !std::isinf(qu)) {
        // Create constraint for the current configuration q0.
        typename SapLimitConstraint<T>::Parameters parameters{
            ql, qu, stiffness, dissipation_time_scale, kBeta};
        problem->AddConstraint(std::make_unique<SapLimitConstraint<T>>(
            tree_index, tree_dof, tree_nv, q0, std::move(parameters)));
      }
    } else {
      // TODO(amcastro-tri): Thus far in Drake we don't have multi-dof joints
      // with limits, only 1-DOF joints have limits. Therefore here throw an
      // exception to ensure that when we implement a multi-dof joint with
      // limits we don't forget to update this code.
      const VectorX<double>& lower_limits = joint.position_lower_limits();
      const VectorX<double>& upper_limits = joint.position_upper_limits();
      if ((lower_limits.array() != -kInf).any() ||
          (upper_limits.array() != kInf).any()) {
        throw std::runtime_error(
            "Limits for joints with more than one degree of freedom are not "
            "supported. You are getting this exception because a new joint "
            "type must have been introduced. "
            "SapDriver::AddLimitConstraints() must be updated to support this "
            "feature. Please file an issue at "
            "https://github.com/RobotLocomotion/drake.");
      }
    }
  }
}

template <typename T>
void SapDriver<T>::AddCouplerConstraints(const systems::Context<T>& context,
                                         SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  // Previous time step positions.
  const VectorX<T> q0 = plant().GetPositions(context);

  // Couplers do not have impulse limits, they are bi-lateral constraints. Each
  // coupler constraint introduces a single constraint equation.
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  const Vector1<T> gamma_lower(-kInfinity);
  const Vector1<T> gamma_upper(kInfinity);

  // Stiffness and dissipation are set so that the constraint is in the
  // "near-rigid" regime, [Castro et al., 2022].
  const Vector1<T> stiffness(kInfinity);
  const Vector1<T> relaxation_time(plant().time_step());

  for (const CouplerConstraintSpecs& info :
       manager().coupler_constraints_specs()) {
    const Joint<T>& joint0 = plant().get_joint(info.joint0_index);
    const Joint<T>& joint1 = plant().get_joint(info.joint1_index);
    const int dof0 = joint0.velocity_start();
    const int dof1 = joint1.velocity_start();
    const TreeIndex tree0 = tree_topology().velocity_to_tree_index(dof0);
    const TreeIndex tree1 = tree_topology().velocity_to_tree_index(dof1);

    // Sanity check.
    DRAKE_DEMAND(tree0.is_valid() && tree1.is_valid());

    // DOFs local to their tree.
    const int tree_dof0 = dof0 - tree_topology().tree_velocities_start(tree0);
    const int tree_dof1 = dof1 - tree_topology().tree_velocities_start(tree1);

    // Constraint function defined as g = q₀ - ρ⋅q₁ - Δq, with ρ the gear ratio
    // and Δq a fixed position offset.
    const Vector1<T> g0(q0[dof0] - info.gear_ratio * q0[dof1] - info.offset);

    // TODO(amcastro-tri): consider exposing this parameter.
    const double beta = 0.1;

    const typename SapHolonomicConstraint<T>::Parameters parameters{
        gamma_lower, gamma_upper, stiffness, relaxation_time, beta};

    if (tree0 == tree1) {
      const int nv = tree_topology().num_tree_velocities(tree0);
      MatrixX<T> J = MatrixX<T>::Zero(1, nv);
      // J = dg/dv
      J(0, tree_dof0) = 1.0;
      J(0, tree_dof1) = -info.gear_ratio;

      problem->AddConstraint(std::make_unique<SapHolonomicConstraint<T>>(
          tree0, g0, J, parameters));
    } else {
      const int nv0 = tree_topology().num_tree_velocities(tree0);
      const int nv1 = tree_topology().num_tree_velocities(tree1);
      MatrixX<T> J0 = MatrixX<T>::Zero(1, nv0);
      MatrixX<T> J1 = MatrixX<T>::Zero(1, nv1);
      J0(0, tree_dof0) = 1.0;
      J1(0, tree_dof1) = -info.gear_ratio;
      problem->AddConstraint(std::make_unique<SapHolonomicConstraint<T>>(
          tree0, tree1, g0, J0, J1, parameters));
    }
  }
}

template <typename T>
void SapDriver<T>::AddDistanceConstraints(const systems::Context<T>& context,
                                          SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  // Distance constraints do not have impulse limits, they are bi-lateral
  // constraints. Each distance constraint introduces a single constraint
  // equation.
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  const Vector1<T> gamma_lower(-kInfinity);
  const Vector1<T> gamma_upper(kInfinity);

  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAp_W(3, nv);
  Matrix3X<T> Jv_WBq_W(3, nv);
  MatrixX<T> Jdistance = MatrixX<T>::Zero(1, nv);

  const Frame<T>& frame_W = plant().world_frame();
  for (const DistanceConstraintSpecs& specs :
       manager().distance_constraints_specs()) {
    const Body<T>& body_A = plant().get_body(specs.body_A);
    const Body<T>& body_B = plant().get_body(specs.body_B);

    const math::RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, body_A);
    const math::RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, body_B);
    const Vector3<T>& p_WP = X_WA * specs.p_AP.cast<T>();
    const Vector3<T>& p_WQ = X_WB * specs.p_BQ.cast<T>();

    // Distance as the norm of p_PQ_W = p_WQ - p_WP
    const Vector3<T> p_PQ_W = p_WQ - p_WP;
    const T d0 = p_PQ_W.norm();
    // Verify the distance did not become ridiculously small. We use the
    // user-specified distance as a reference.
    constexpr double kMinimumDistance = 1.0e-7;
    constexpr double kRelativeDistance = 1.0e-2;
    if (d0 < kMinimumDistance + kRelativeDistance * specs.distance) {
      const std::string msg = fmt::format(
          "The distance between bodies '{}' and '{}' is: {}. This is "
          "nonphysically small when compared to the free length of the "
          "constraint, {}. ",
          body_A.name(), body_B.name(), d0, specs.distance);
      throw std::logic_error(msg);
    }
    const Vector3<T> p_hat_W = p_PQ_W / d0;

    DRAKE_DEMAND(body_A.index() != body_B.index());

    // Dense Jacobian.
    // d(distance)/dt = Jdistance * v.
    manager().internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_A.body_frame(), frame_W, p_WP,
        frame_W, frame_W, &Jv_WAp_W);
    manager().internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_B.body_frame(), frame_W, p_WQ,
        frame_W, frame_W, &Jv_WBq_W);
    Jdistance = p_hat_W.transpose() * (Jv_WBq_W - Jv_WAp_W);

    const T dissipation_time_scale = specs.damping / specs.stiffness;

    // TODO(amcastro-tri): consider exposing this parameter.
    const double beta = 0.1;
    const typename SapHolonomicConstraint<T>::Parameters parameters{
        gamma_lower, gamma_upper, Vector1<T>(specs.stiffness),
        Vector1<T>(dissipation_time_scale), beta};

    const TreeIndex treeA_index =
        tree_topology().body_to_tree_index(specs.body_A);
    const TreeIndex treeB_index =
        tree_topology().body_to_tree_index(specs.body_B);

    // Sanity check at least one body is not the world.
    DRAKE_DEMAND(treeA_index.is_valid() || treeB_index.is_valid());

    // Both bodies A and B belong to the same tree or one of them is the world.
    const bool single_tree = !treeA_index.is_valid() ||
                             !treeB_index.is_valid() ||
                             treeA_index == treeB_index;

    // Constraint function at current time step.
    const Vector1<T> g0(d0 - specs.distance);
    if (single_tree) {
      const TreeIndex tree_index =
          treeA_index.is_valid() ? treeA_index : treeB_index;
      const MatrixX<T> J = Jdistance.middleCols(
          tree_topology().tree_velocities_start(tree_index),
          tree_topology().num_tree_velocities(tree_index));

      problem->AddConstraint(std::make_unique<SapHolonomicConstraint<T>>(
          tree_index, g0, J, parameters));
    } else {
      const MatrixX<T> JA = Jdistance.middleCols(
          tree_topology().tree_velocities_start(treeA_index),
          tree_topology().num_tree_velocities(treeA_index));
      const MatrixX<T> JB = Jdistance.middleCols(
          tree_topology().tree_velocities_start(treeB_index),
          tree_topology().num_tree_velocities(treeB_index));
      problem->AddConstraint(std::make_unique<SapHolonomicConstraint<T>>(
          treeA_index, treeB_index, g0, JA, JB, parameters));
    }
  }
}

template <typename T>
void SapDriver<T>::AddBallConstraints(
    const systems::Context<T>& context,
    contact_solvers::internal::SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  // Ball constraints do not have impulse limits, they are bi-lateral
  // constraints. Each ball constraint introduces three constraint
  // equations.
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  const Vector3<T> gamma_lower(-kInfinity, -kInfinity, -kInfinity);
  const Vector3<T> gamma_upper(kInfinity, kInfinity, kInfinity);

  // Stiffness and dissipation are set so that the constraint is in the
  // "near-rigid" regime, [Castro et al., 2022].
  const Vector3<T> stiffness(kInfinity, kInfinity, kInfinity);
  const Vector3<T> relaxation_time = plant().time_step() * Vector3<T>::Ones();

  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAp_W(3, nv);
  Matrix3X<T> Jv_WBq_W(3, nv);
  MatrixX<T> Jv_ApBq_W = MatrixX<T>::Zero(3, nv);

  const Frame<T>& frame_W = plant().world_frame();
  for (const BallConstraintSpecs& specs :
       manager().ball_constraints_specs()) {
    const Body<T>& body_A = plant().get_body(specs.body_A);
    const Body<T>& body_B = plant().get_body(specs.body_B);

    const math::RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, body_A);
    const math::RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, body_B);
    const Vector3<T> p_WP = X_WA * specs.p_AP.cast<T>();
    const Vector3<T> p_WQ = X_WB * specs.p_BQ.cast<T>();

    const Vector3<T> p_PQ_W = p_WQ - p_WP;

    // Dense Jacobian.
    // d(p_PQ_W)/dt = Jv_ApBq_W * v.
    manager().internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_A.body_frame(), frame_W, p_WP,
        frame_W, frame_W, &Jv_WAp_W);
    manager().internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_B.body_frame(), frame_W, p_WQ,
        frame_W, frame_W, &Jv_WBq_W);
    Jv_ApBq_W = (Jv_WBq_W - Jv_WAp_W);

    // TODO(amcastro-tri): consider exposing this parameter.
    const double beta = 0.1;
    const typename SapHolonomicConstraint<T>::Parameters parameters{
        gamma_lower, gamma_upper, stiffness, relaxation_time, beta};

    const TreeIndex treeA_index =
        tree_topology().body_to_tree_index(specs.body_A);
    const TreeIndex treeB_index =
        tree_topology().body_to_tree_index(specs.body_B);

    // TODO(joemasterjohn): Move this exception up to the plant level so that it
    // fails as fast as possible. Currently, the earliest this can happen is
    // in MbP::Finalize() after the topology has been finalized.
    if (!treeA_index.is_valid() && !treeB_index.is_valid()) {
      const std::string msg = fmt::format(
          "Creating a ball Constraint between bodies '{}' and '{}' where both "
          "are welded to the world is not allowed.",
          body_A.name(), body_B.name());
      throw std::runtime_error(msg);
    }

    // Both bodies A and B belong to the same tree or one of them is the world.
    const bool single_tree = !treeA_index.is_valid() ||
                             !treeB_index.is_valid() ||
                             treeA_index == treeB_index;

    // Constraint function at current time step.
    const Vector3<T> g0 = p_PQ_W;
    if (single_tree) {
      const TreeIndex tree_index =
          treeA_index.is_valid() ? treeA_index : treeB_index;
      const MatrixX<T> J = Jv_ApBq_W.middleCols(
          tree_topology().tree_velocities_start(tree_index),
          tree_topology().num_tree_velocities(tree_index));

      problem->AddConstraint(std::make_unique<SapHolonomicConstraint<T>>(
          tree_index, g0, std::move(J), parameters));
    } else {
      const MatrixX<T> JA = Jv_ApBq_W.middleCols(
          tree_topology().tree_velocities_start(treeA_index),
          tree_topology().num_tree_velocities(treeA_index));
      const MatrixX<T> JB = Jv_ApBq_W.middleCols(
          tree_topology().tree_velocities_start(treeB_index),
          tree_topology().num_tree_velocities(treeB_index));
      problem->AddConstraint(std::make_unique<SapHolonomicConstraint<T>>(
          treeA_index, treeB_index, g0, std::move(JA), std::move(JB),
          parameters));
    }
  }
}

template <typename T>
void SapDriver<T>::CalcContactProblemCache(
    const systems::Context<T>& context, ContactProblemCache<T>* cache) const {
  SapContactProblem<T>& problem = *cache->sap_problem;
  std::vector<MatrixX<T>> A;
  CalcLinearDynamicsMatrix(context, &A);
  VectorX<T> v_star;
  CalcFreeMotionVelocities(context, &v_star);
  problem.Reset(std::move(A), std::move(v_star));
  // N.B. All contact constraints must be added before any other constraint
  // types. This driver assumes this ordering of the constraints in order to
  // extract contact impulses for reporting contact results.
  // Do not change this order here!
  cache->R_WC = AddContactConstraints(context, &problem);
  AddLimitConstraints(context, problem.v_star(), &problem);
  AddCouplerConstraints(context, &problem);
  AddDistanceConstraints(context, &problem);
  AddBallConstraints(context, &problem);
}

template <typename T>
void SapDriver<T>::PackContactSolverResults(
    const systems::Context<T>& context, const SapContactProblem<T>& problem,
    int num_contacts, const SapSolverResults<T>& sap_results,
    ContactSolverResults<T>* contact_results) const {
  DRAKE_DEMAND(contact_results != nullptr);
  contact_results->Resize(sap_results.v.size(), num_contacts);
  contact_results->v_next = sap_results.v;
  // The driver adds all contact constraints first and therefore we know the
  // head of the impulses corresponds to contact impulses.
  const Eigen::VectorBlock<const VectorX<T>> contact_impulses =
      sap_results.gamma.head(3 * num_contacts);
  const Eigen::VectorBlock<const VectorX<T>> contact_velocities =
      sap_results.vc.head(3 * num_contacts);
  const double time_step = plant().time_step();
  ExtractNormal(contact_impulses, &contact_results->fn);
  ExtractTangent(contact_impulses, &contact_results->ft);
  contact_results->fn /= time_step;
  contact_results->ft /= time_step;
  ExtractNormal(contact_velocities, &contact_results->vn);
  ExtractTangent(contact_velocities, &contact_results->vt);

  auto& tau_contact = contact_results->tau_contact;
  tau_contact.setZero();
  for (int i = 0; i < num_contacts; ++i) {
    const SapConstraint<T>& c = problem.get_constraint(i);
    {
      const MatrixBlock<T>& Jic = c.first_clique_jacobian();
      const auto impulse = contact_impulses.template segment<3>(3 * i);
      VectorX<T> tau_clique = VectorX<T>::Zero(Jic.cols());
      Jic.TransposeAndMultiplyAndAddTo(impulse, &tau_clique);
      AddCliqueContribution(context, c.first_clique(), tau_clique,
                            &tau_contact);
    }

    if (c.num_cliques() == 2) {
      const MatrixBlock<T>& Jic = c.second_clique_jacobian();
      const auto impulse = contact_impulses.template segment<3>(3 * i);
      VectorX<T> tau_clique = VectorX<T>::Zero(Jic.cols());
      Jic.TransposeAndMultiplyAndAddTo(impulse, &tau_clique);
      AddCliqueContribution(context, c.second_clique(), tau_clique,
                            &tau_contact);
    }
  }
  tau_contact /= time_step;
}

template <typename T>
void SapDriver<T>::AddCliqueContribution(
    const systems::Context<T>& context, int clique,
    const Eigen::Ref<const VectorX<T>>& clique_values,
    EigenPtr<VectorX<T>> values) const {
  if (clique >= tree_topology().num_trees()) {
    const DeformableDriver<double>* deformable_driver =
        manager().deformable_driver_.get();
    DRAKE_THROW_UNLESS(deformable_driver != nullptr);
    if constexpr (std::is_same_v<T, double>) {
      const int num_deformable_dofs = values->size() - plant().num_velocities();
      Eigen::Ref<VectorX<T>> deformable_values =
          values->tail(num_deformable_dofs);
      DeformableBodyIndex body_index(clique - tree_topology().num_trees());
      deformable_driver->EvalParticipatingVelocityMultiplexer(context)
          .Demultiplex(&deformable_values, body_index) += clique_values;
    } else {
      /* For non-double scalars, we can't have `deformable_driver != nullptr`,
       so we won't reach here. */
      DRAKE_UNREACHABLE();
    }
  } else {
    const TreeIndex t(clique);
    const int v_start = tree_topology().tree_velocities_start(t);
    const int nv = tree_topology().num_tree_velocities(t);
    values->segment(v_start, nv) += clique_values;
  }
}

template <typename T>
void SapDriver<T>::CalcContactSolverResults(
    const systems::Context<T>& context,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  const ContactProblemCache<T>& contact_problem_cache =
      EvalContactProblemCache(context);
  const SapContactProblem<T>& sap_problem = *contact_problem_cache.sap_problem;

  // We use the velocity stored in the current context as initial guess.
  const VectorX<T>& x0 =
      context.get_discrete_state(manager().multibody_state_index()).value();
  VectorX<T> v0 = x0.bottomRows(this->plant().num_velocities());
  if constexpr (std::is_same_v<T, double>) {
    if (manager().deformable_driver_ != nullptr) {
      const VectorX<double> deformable_v0 =
          manager().deformable_driver_->EvalParticipatingVelocities(context);
      const int rigid_dofs = v0.size();
      const int deformable_dofs = deformable_v0.size();
      v0.conservativeResize(rigid_dofs + deformable_dofs);
      v0.tail(deformable_dofs) = deformable_v0;
    }
  }

  // Solve contact problem.
  SapSolver<T> sap;
  sap.set_parameters(sap_parameters_);
  SapSolverResults<T> sap_results;
  const SapSolverStatus status =
      sap.SolveWithGuess(sap_problem, v0, &sap_results);
  if (status != SapSolverStatus::kSuccess) {
    const std::string msg = fmt::format(
        "The SAP solver failed to converge at simulation time = {}. "
        "Reasons for divergence and possible solutions include:\n"
        "  1. Externally applied actuation values diverged due to external "
        "     reasons to the solver. Revise your control logic.\n"
        "  2. External force elements such as spring or bushing elements can "
        "     lead to unstable temporal dynamics if too stiff. Revise your "
        "     model and consider whether these forces can be better modeled "
        "     using one of SAP's compliant constraints. E.g., use a distance "
        "     constraint instead of a spring element.\n"
        "  3. Numerical ill conditioning of the model caused by, for instance, "
        "     extremely large mass ratios. Revise your model and consider "
        "     whether very small objects can be removed or welded to larger "
        "     objects in the model."
        "  4. Some other cause. You may want to use Stack Overflow (#drake "
        "     tag) to request some assistance.",
        context.get_time());
    throw std::runtime_error(msg);
  }

  const std::vector<DiscreteContactPair<T>>& discrete_pairs =
      manager().EvalDiscreteContactPairs(context);
  const int num_contacts = discrete_pairs.size();

  PackContactSolverResults(context, sap_problem, num_contacts, sap_results,
                           results);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::SapDriver);
