#include "drake/multibody/plant/sap_driver.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/unused.h"
#include "drake/multibody/contact_solvers/contact_configuration.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/sap/sap_ball_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_coupler_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_distance_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_fixed_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_limit_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_pd_controller_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/contact_solvers/sap/sap_weld_constraint.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/slicing_and_indexing.h"

using drake::geometry::GeometryId;
using drake::math::RotationMatrix;
using drake::multibody::contact_solvers::internal::ContactConfiguration;
using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::ExtractNormal;
using drake::multibody::contact_solvers::internal::ExtractTangent;
using drake::multibody::contact_solvers::internal::FixedConstraintKinematics;
using drake::multibody::contact_solvers::internal::MakeContactConfiguration;
using drake::multibody::contact_solvers::internal::MatrixBlock;
using drake::multibody::contact_solvers::internal::SapBallConstraint;
using drake::multibody::contact_solvers::internal::SapConstraint;
using drake::multibody::contact_solvers::internal::SapConstraintJacobian;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapCouplerConstraint;
using drake::multibody::contact_solvers::internal::SapDistanceConstraint;
using drake::multibody::contact_solvers::internal::SapFixedConstraint;
using drake::multibody::contact_solvers::internal::SapFrictionConeConstraint;
using drake::multibody::contact_solvers::internal::SapHolonomicConstraint;
using drake::multibody::contact_solvers::internal::SapHuntCrossleyApproximation;
using drake::multibody::contact_solvers::internal::SapHuntCrossleyConstraint;
using drake::multibody::contact_solvers::internal::SapLimitConstraint;
using drake::multibody::contact_solvers::internal::SapPdControllerConstraint;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;
using drake::multibody::contact_solvers::internal::SapWeldConstraint;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
SapDriver<T>::SapDriver(const CompliantContactManager<T>* manager,
                        double near_rigid_threshold)
    : manager_(manager), near_rigid_threshold_(near_rigid_threshold) {
  DRAKE_DEMAND(manager != nullptr);
  DRAKE_DEMAND(near_rigid_threshold >= 0.0);
}

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

  const auto& sap_solver_results_cache_entry =
      mutable_manager->DeclareCacheEntry(
          "SAP solver results",
          systems::ValueProducer(this, &SapDriver<T>::CalcSapSolverResults),
          state_input_and_parameters);
  sap_results_ = sap_solver_results_cache_entry.cache_index();
}

template <typename T>
const SapSolverResults<T>& SapDriver<T>::EvalSapSolverResults(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(sap_results_)
      .template Eval<SapSolverResults<T>>(context);
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
  M.diagonal() += plant().time_step() * plant().EvalJointDampingCache(context);

  for (TreeIndex t(0); t < tree_topology().num_trees(); ++t) {
    const int tree_start_in_v = tree_topology().tree_velocities_start_in_v(t);
    const int tree_nv = tree_topology().num_tree_velocities(t);
    (*A)[t] = M.block(tree_start_in_v, tree_start_in_v, tree_nv, tree_nv);
  }

  if constexpr (std::is_same_v<T, double>) {
    if (manager().deformable_driver() != nullptr) {
      manager().deformable_driver()->AppendLinearDynamicsMatrix(context, A);
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
      manager()
          .EvalAccelerationsDueToNonConstraintForcesCache(context)
          .get_vdot();
  const double dt = this->plant().time_step();
  const VectorX<T>& x0 =
      context.get_discrete_state(manager().multibody_state_index()).value();
  const auto v0 = x0.bottomRows(this->plant().num_velocities());
  if constexpr (std::is_same_v<T, double>) {
    if (manager().deformable_driver() != nullptr) {
      const VectorX<T>& deformable_v_star =
          manager().deformable_driver()->EvalParticipatingFreeMotionVelocities(
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
  DRAKE_DEMAND(plant().get_discrete_contact_approximation() !=
               DiscreteContactApproximation::kTamsi);

  // Parameters used by SAP to estimate regularization, see [Castro et al.,
  // 2021].
  // TODO(amcastro-tri): consider exposing these parameters.
  constexpr double sigma = 1.0e-3;

  const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs =
      manager().EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();

  // Quick no-op exit.
  if (num_contacts == 0) return std::vector<RotationMatrix<T>>();

  std::vector<RotationMatrix<T>> R_WC;
  R_WC.reserve(num_contacts);
  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& pair = contact_pairs[icontact];

    const T stiffness = pair.stiffness;
    const T dissipation_time_scale = pair.dissipation_time_scale;
    const T damping = pair.damping;
    const T friction = pair.friction_coefficient;
    const auto& jacobian_blocks = pair.jacobian;
    const double beta = near_rigid_threshold_;

    auto make_sap_parameters = [&]() {
      return typename SapFrictionConeConstraint<T>::Parameters{
          friction, stiffness, dissipation_time_scale, beta, sigma};
    };

    auto make_hunt_crossley_parameters = [&]() {
      const double vs = plant().stiction_tolerance();
      SapHuntCrossleyApproximation model;
      switch (plant().get_discrete_contact_approximation()) {
        case DiscreteContactApproximation::kSimilar:
          model = SapHuntCrossleyApproximation::kSimilar;
          break;
        case DiscreteContactApproximation::kLagged:
          model = SapHuntCrossleyApproximation::kLagged;
          break;
        default:
          DRAKE_UNREACHABLE();
      }
      return typename SapHuntCrossleyConstraint<T>::Parameters{
          model, friction, stiffness, damping, vs, sigma};
    };

    R_WC.push_back(pair.R_WC);

    if (jacobian_blocks.size() == 1) {
      SapConstraintJacobian<T> J(jacobian_blocks[0].tree,
                                 std::move(jacobian_blocks[0].J));
      if (plant().get_discrete_contact_approximation() ==
          DiscreteContactApproximation::kSap) {
        problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<T>>(
            MakeContactConfiguration(pair), std::move(J),
            make_sap_parameters()));
      } else {
        ContactConfiguration<T> contact_configuration =
            MakeContactConfiguration(pair);
        DRAKE_DEMAND(
            !std::isnan(ExtractDoubleOrThrow(contact_configuration.vn)));
        DRAKE_DEMAND(
            !std::isnan(ExtractDoubleOrThrow(contact_configuration.fe)));
        DRAKE_DEMAND(
            !std::isnan(ExtractDoubleOrThrow(contact_configuration.phi)));
        problem->AddConstraint(std::make_unique<SapHuntCrossleyConstraint<T>>(
            std::move(contact_configuration), std::move(J),
            make_hunt_crossley_parameters()));
      }
    } else {
      SapConstraintJacobian<T> J(
          jacobian_blocks[0].tree, std::move(jacobian_blocks[0].J),
          jacobian_blocks[1].tree, std::move(jacobian_blocks[1].J));
      if (plant().get_discrete_contact_approximation() ==
          DiscreteContactApproximation::kSap) {
        problem->AddConstraint(std::make_unique<SapFrictionConeConstraint<T>>(
            MakeContactConfiguration(pair), std::move(J),
            make_sap_parameters()));
      } else {
        problem->AddConstraint(std::make_unique<SapHuntCrossleyConstraint<T>>(
            MakeContactConfiguration(pair), std::move(J),
            make_hunt_crossley_parameters()));
      }
    }
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
          tree_topology().tree_velocities_start_in_v(tree_index);
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

  const std::map<MultibodyConstraintId, bool>& constraint_active_status =
      manager().GetConstraintActiveStatus(context);

  for (const auto& [id, info] : manager().coupler_constraints_specs()) {
    // Skip this constraint if it is not active.
    if (!constraint_active_status.at(id)) continue;
    const Joint<T>& joint0 = plant().get_joint(info.joint0_index);
    const Joint<T>& joint1 = plant().get_joint(info.joint1_index);
    const int dof0 = joint0.velocity_start();
    const int dof1 = joint1.velocity_start();
    const TreeIndex tree0 = tree_topology().velocity_to_tree_index(dof0);
    const TreeIndex tree1 = tree_topology().velocity_to_tree_index(dof1);

    // Sanity check.
    DRAKE_DEMAND(tree0.is_valid() && tree1.is_valid());

    // DOFs local to their tree.
    const int tree_dof0 =
        dof0 - tree_topology().tree_velocities_start_in_v(tree0);
    const int tree_dof1 =
        dof1 - tree_topology().tree_velocities_start_in_v(tree1);

    const int tree_nv0 = tree_topology().num_tree_velocities(tree0);
    const int tree_nv1 = tree_topology().num_tree_velocities(tree1);

    const typename SapCouplerConstraint<T>::Kinematics kinematics{
        tree0,           tree_dof0,  tree_nv0, joint0.GetOnePosition(context),
        tree1,           tree_dof1,  tree_nv1, joint1.GetOnePosition(context),
        info.gear_ratio, info.offset};

    problem->AddConstraint(
        std::make_unique<SapCouplerConstraint<T>>(std::move(kinematics)));
  }
}

template <typename T>
void SapDriver<T>::AddDistanceConstraints(const systems::Context<T>& context,
                                          SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAp_W(3, nv);
  Matrix3X<T> Jv_WBq_W(3, nv);
  Matrix3X<T> Jv_ApBq_W(3, nv);

  const Frame<T>& frame_W = plant().world_frame();

  const std::map<MultibodyConstraintId, bool>& constraint_active_status =
      manager().GetConstraintActiveStatus(context);

  for (const auto& [id, spec] : manager().distance_constraints_specs()) {
    // skip this constraint if it is not active.
    if (!constraint_active_status.at(id)) continue;

    const RigidBody<T>& body_A = plant().get_body(spec.body_A);
    const RigidBody<T>& body_B = plant().get_body(spec.body_B);
    DRAKE_DEMAND(body_A.index() != body_B.index());

    const math::RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, body_A);
    const math::RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, body_B);
    const Vector3<T> p_WP = X_WA * spec.p_AP.template cast<T>();
    const Vector3<T> p_AP_W = X_WA.rotation() * spec.p_AP.template cast<T>();
    const Vector3<T> p_WQ = X_WB * spec.p_BQ.template cast<T>();
    const Vector3<T> p_BQ_W = X_WB.rotation() * spec.p_BQ.template cast<T>();

    // Jacobian for the velocity of point Q (on body B) relative to point P (on
    // body B).
    manager().internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_A.body_frame(), frame_W, p_WP,
        frame_W, frame_W, &Jv_WAp_W);
    manager().internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_B.body_frame(), frame_W, p_WQ,
        frame_W, frame_W, &Jv_WBq_W);
    Jv_ApBq_W = Jv_WBq_W - Jv_WAp_W;

    // Jacobian for the relative velocity v_PQ_W, as required by
    // SapDistanceConstraint.
    auto make_constraint_jacobian = [this, &Jv_ApBq_W](BodyIndex bodyA,
                                                       BodyIndex bodyB) {
      const TreeIndex treeA_index = tree_topology().body_to_tree_index(bodyA);
      const TreeIndex treeB_index = tree_topology().body_to_tree_index(bodyB);
      const bool treeA_has_dofs = tree_topology().tree_has_dofs(treeA_index);
      const bool treeB_has_dofs = tree_topology().tree_has_dofs(treeB_index);

      // Sanity check at least one body is not World or anchored to World.
      DRAKE_DEMAND(treeA_has_dofs || treeB_has_dofs);

      // Both bodies A and B belong to the same tree or one of them is the
      // world.
      const bool single_tree =
          !treeA_has_dofs || !treeB_has_dofs || treeA_index == treeB_index;

      if (single_tree) {
        const TreeIndex tree_index = treeA_has_dofs ? treeA_index : treeB_index;
        MatrixX<T> Jtree = Jv_ApBq_W.middleCols(
            tree_topology().tree_velocities_start_in_v(tree_index),
            tree_topology().num_tree_velocities(tree_index));
        return SapConstraintJacobian<T>(tree_index, std::move(Jtree));
      } else {
        MatrixX<T> JA = Jv_ApBq_W.middleCols(
            tree_topology().tree_velocities_start_in_v(treeA_index),
            tree_topology().num_tree_velocities(treeA_index));
        MatrixX<T> JB = Jv_ApBq_W.middleCols(
            tree_topology().tree_velocities_start_in_v(treeB_index),
            tree_topology().num_tree_velocities(treeB_index));
        return SapConstraintJacobian<T>(treeA_index, std::move(JA), treeB_index,
                                        std::move(JB));
      }
    };

    const typename SapDistanceConstraint<T>::Kinematics kinematics(
        spec.body_A, p_WP, p_AP_W, spec.body_B, p_WQ, p_BQ_W, spec.distance,
        make_constraint_jacobian(spec.body_A, spec.body_B));

    const typename SapDistanceConstraint<T>::ComplianceParameters parameters(
        spec.stiffness, spec.damping);

    problem->AddConstraint(std::make_unique<SapDistanceConstraint<T>>(
        std::move(kinematics), std::move(parameters)));
  }
}

template <typename T>
void SapDriver<T>::AddBallConstraints(
    const systems::Context<T>& context,
    contact_solvers::internal::SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAp_W(3, nv);
  Matrix3X<T> Jv_WBq_W(3, nv);
  Matrix3X<T> Jv_ApBq_W(3, nv);

  const Frame<T>& frame_W = plant().world_frame();

  const std::map<MultibodyConstraintId, bool>& constraint_active_status =
      manager().GetConstraintActiveStatus(context);

  for (const auto& [id, spec] : manager().ball_constraints_specs()) {
    // skip this constraint if it is not active.
    if (!constraint_active_status.at(id)) continue;

    const RigidBody<T>& body_A = plant().get_body(spec.body_A);
    const RigidBody<T>& body_B = plant().get_body(spec.body_B);

    const math::RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, body_A);
    const math::RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, body_B);
    const Vector3<T> p_WP = X_WA * spec.p_AP.template cast<T>();
    const Vector3<T> p_AP_W = X_WA.rotation() * spec.p_AP.template cast<T>();
    const Vector3<T> p_WQ = X_WB * spec.p_BQ.template cast<T>();
    const Vector3<T> p_BQ_W = X_WB.rotation() * spec.p_BQ.template cast<T>();

    // Dense Jacobian.
    // d(p_PQ_W)/dt = Jv_ApBq_W * v.
    manager().internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_A.body_frame(), frame_W, p_WP,
        frame_W, frame_W, &Jv_WAp_W);
    manager().internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_B.body_frame(), frame_W, p_WQ,
        frame_W, frame_W, &Jv_WBq_W);
    Jv_ApBq_W = (Jv_WBq_W - Jv_WAp_W);

    // Jacobian for the relative velocity v_PQ_W, as required by
    // SapDistanceConstraint.
    auto make_constraint_jacobian = [this, &Jv_ApBq_W, &body_A, &body_B]() {
      const TreeIndex treeA_index =
          tree_topology().body_to_tree_index(body_A.index());
      const TreeIndex treeB_index =
          tree_topology().body_to_tree_index(body_B.index());
      const bool treeA_has_dofs = tree_topology().tree_has_dofs(treeA_index);
      const bool treeB_has_dofs = tree_topology().tree_has_dofs(treeB_index);

      // TODO(joemasterjohn): Move this exception up to the plant level so that
      // it fails as fast as possible. Currently, the earliest this can happen
      // is in MbP::Finalize() after the topology has been finalized.
      if (!treeA_has_dofs && !treeB_has_dofs) {
        const std::string msg = fmt::format(
            "Creating a ball Constraint between bodies '{}' and '{}' where "
            "both are welded to the world is not allowed.",
            body_A.name(), body_B.name());
        throw std::runtime_error(msg);
      }

      // Both bodies A and B belong to the same tree or one of them is the
      // world.
      const bool single_tree =
          !treeA_has_dofs || !treeB_has_dofs || treeA_index == treeB_index;

      if (single_tree) {
        const TreeIndex tree_index = treeA_has_dofs ? treeA_index : treeB_index;
        MatrixX<T> Jtree = Jv_ApBq_W.middleCols(
            tree_topology().tree_velocities_start_in_v(tree_index),
            tree_topology().num_tree_velocities(tree_index));
        return SapConstraintJacobian<T>(tree_index, std::move(Jtree));
      } else {
        MatrixX<T> JA = Jv_ApBq_W.middleCols(
            tree_topology().tree_velocities_start_in_v(treeA_index),
            tree_topology().num_tree_velocities(treeA_index));
        MatrixX<T> JB = Jv_ApBq_W.middleCols(
            tree_topology().tree_velocities_start_in_v(treeB_index),
            tree_topology().num_tree_velocities(treeB_index));
        return SapConstraintJacobian<T>(treeA_index, std::move(JA), treeB_index,
                                        std::move(JB));
      }
    };

    const typename SapBallConstraint<T>::Kinematics kinematics(
        spec.body_A, p_WP, p_AP_W, spec.body_B, p_WQ, p_BQ_W,
        make_constraint_jacobian());

    problem->AddConstraint(
        std::make_unique<SapBallConstraint<T>>(std::move(kinematics)));
  }
}

template <typename T>
void SapDriver<T>::AddWeldConstraints(
    const systems::Context<T>& context,
    contact_solvers::internal::SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  const int nv = plant().num_velocities();
  Matrix6X<T> J_WAm(6, nv);
  Matrix6X<T> J_WBm(6, nv);
  MatrixX<T> J_W_AmBm = MatrixX<T>::Zero(6, nv);

  const Frame<T>& frame_W = plant().world_frame();

  const std::map<MultibodyConstraintId, bool>& constraint_active_status =
      manager().GetConstraintActiveStatus(context);

  for (const auto& [id, spec] : manager().weld_constraints_specs()) {
    // skip this constraint if it is not active.
    if (!constraint_active_status.at(id)) continue;

    const RigidBody<T>& body_A = plant().get_body(spec.body_A);
    const RigidBody<T>& body_B = plant().get_body(spec.body_B);

    const math::RigidTransform<T>& X_WA = body_A.EvalPoseInWorld(context);
    const math::RigidTransform<T>& X_WB = body_B.EvalPoseInWorld(context);
    const math::RigidTransform<T> X_WP = X_WA * spec.X_AP.template cast<T>();
    const math::RigidTransform<T> X_WQ = X_WB * spec.X_BQ.template cast<T>();
    const Vector3<T> p_AP_W =
        X_WA.rotation() * spec.X_AP.translation().template cast<T>();
    const Vector3<T> p_BQ_W =
        X_WB.rotation() * spec.X_BQ.translation().template cast<T>();

    // Let M be the midpoint of P and Q.
    const Vector3<T> p_WM = 0.5 * (X_WP.translation() + X_WQ.translation());
    const Vector3<T> p_AM = p_WM - X_WA.translation();
    const Vector3<T> p_BM = p_WM - X_WB.translation();

    // Dense Jacobian.
    manager().internal_tree().CalcJacobianSpatialVelocity(
        context, JacobianWrtVariable::kV, body_A.body_frame(), p_AM, frame_W,
        frame_W, &J_WAm);
    manager().internal_tree().CalcJacobianSpatialVelocity(
        context, JacobianWrtVariable::kV, body_B.body_frame(), p_BM, frame_W,
        frame_W, &J_WBm);
    J_W_AmBm = (J_WBm - J_WAm);

    // TODO(joemasterjohn) consolidate make_jacobian for use by other
    // constraints that need the same jacobian computed here.
    auto make_constraint_jacobian = [this, &J_W_AmBm](
                                        const RigidBody<T>& bodyA,
                                        const RigidBody<T>& bodyB) {
      const TreeIndex treeA_index =
          tree_topology().body_to_tree_index(bodyA.index());
      const TreeIndex treeB_index =
          tree_topology().body_to_tree_index(bodyB.index());
      const bool treeA_has_dofs = tree_topology().tree_has_dofs(treeA_index);
      const bool treeB_has_dofs = tree_topology().tree_has_dofs(treeB_index);

      // TODO(joemasterjohn): Move this exception up to the plant level so
      //  that it fails as fast as possible. Currently, the earliest this can
      //  happen is in MbP::Finalize() after the topology has been finalized.
      if (!treeA_has_dofs && !treeB_has_dofs) {
        const std::string msg = fmt::format(
            "Creating a weld constraint between bodies '{}' and '{}' where "
            "both are welded to the world is not allowed.",
            bodyA.name(), bodyB.name());
        throw std::logic_error(msg);
      }

      // Both bodies A and B belong to the same tree or one of them is the
      // world.
      const bool single_tree =
          !treeA_has_dofs || !treeB_has_dofs || treeA_index == treeB_index;

      if (single_tree) {
        const TreeIndex tree_index = treeA_has_dofs ? treeA_index : treeB_index;
        MatrixX<T> Jtree_W = J_W_AmBm.middleCols(
            tree_topology().tree_velocities_start_in_v(tree_index),
            tree_topology().num_tree_velocities(tree_index));
        return SapConstraintJacobian<T>(tree_index, std::move(Jtree_W));
      } else {
        MatrixX<T> JA_W = J_W_AmBm.middleCols(
            tree_topology().tree_velocities_start_in_v(treeA_index),
            tree_topology().num_tree_velocities(treeA_index));
        MatrixX<T> JB_W = J_W_AmBm.middleCols(
            tree_topology().tree_velocities_start_in_v(treeB_index),
            tree_topology().num_tree_velocities(treeB_index));
        return SapConstraintJacobian<T>(treeA_index, std::move(JA_W),
                                        treeB_index, std::move(JB_W));
      }
    };

    const typename SapWeldConstraint<T>::Kinematics kinematics(
        spec.body_A, X_WP, p_AP_W, spec.body_B, X_WQ, p_BQ_W,
        make_constraint_jacobian(body_A, body_B));

    problem->AddConstraint(
        std::make_unique<SapWeldConstraint<T>>(std::move(kinematics)));
  }
}

template <typename T>
void SapDriver<T>::AddPdControllerConstraints(
    const systems::Context<T>& context, SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);

  // Do nothing if not PD controllers were specified.
  if (plant().num_actuators() == 0) return;

  // Desired positions & velocities.
  const int num_actuators = plant().num_actuators();
  // TODO(amcastro-tri): makes these EvalFoo() instead to avoid heap
  // allocations.
  const VectorX<T> desired_state = manager_->AssembleDesiredStateInput(context);
  const VectorX<T> feed_forward_actuation =
      manager_->AssembleActuationInput(context);

  for (JointActuatorIndex actuator_index(0);
       actuator_index < plant().num_actuators(); ++actuator_index) {
    const JointActuator<T>& actuator =
        plant().get_joint_actuator(actuator_index);
    if (actuator.has_controller()) {
      const Joint<T>& joint = actuator.joint();
      // There is no point in modeling PD controllers if the joint is locked.
      // Therefore we do not add these constraints and actuation due to PD
      // controllers on locked joints is considered to be zero.
      if (!joint.is_locked(context)) {
        const double effort_limit = actuator.effort_limit();
        const T& qd = desired_state[actuator.index()];
        const T& vd = desired_state[num_actuators + actuator.index()];
        const T& u0 = feed_forward_actuation[actuator.index()];

        const T& q0 = joint.GetOnePosition(context);
        const int dof = joint.velocity_start();
        const TreeIndex tree = tree_topology().velocity_to_tree_index(dof);
        const int tree_dof =
            dof - tree_topology().tree_velocities_start_in_v(tree);
        const int tree_nv = tree_topology().num_tree_velocities(tree);

        // Controller gains.
        const PdControllerGains& gains = actuator.get_controller_gains();
        const T& Kp = gains.p;
        const T& Kd = gains.d;

        typename SapPdControllerConstraint<T>::Parameters parameters{
            Kp, Kd, effort_limit};
        typename SapPdControllerConstraint<T>::Configuration configuration{
            tree, tree_dof, tree_nv, q0, qd, vd, u0};

        problem->AddConstraint(std::make_unique<SapPdControllerConstraint<T>>(
            std::move(configuration), std::move(parameters)));
      }
    }
  }
}

template <typename T>
void SapDriver<T>::AddFixedConstraints(
    const systems::Context<T>& context,
    contact_solvers::internal::SapContactProblem<T>* problem) const {
  DRAKE_DEMAND(problem != nullptr);
  if constexpr (std::is_same_v<T, double>) {
    if (manager().deformable_driver() != nullptr) {
      std::vector<FixedConstraintKinematics<T>> kinematics;
      manager()
          .deformable_driver()
          ->AppendDeformableRigidFixedConstraintKinematics(context,
                                                           &kinematics);
      for (FixedConstraintKinematics<T>& k : kinematics) {
        problem->AddConstraint(
            std::make_unique<SapFixedConstraint<T>>(std::move(k)));
      }
    }
  } else {
    unused(context, problem);
  }
}

template <typename T>
void SapDriver<T>::CalcContactProblemCache(
    const systems::Context<T>& context, ContactProblemCache<T>* cache) const {
  std::vector<MatrixX<T>> A;
  CalcLinearDynamicsMatrix(context, &A);
  VectorX<T> v_star;
  CalcFreeMotionVelocities(context, &v_star);
  const int num_rigid_bodies = plant().num_bodies();
  const int num_deformable_bodies =
      (manager().deformable_driver() == nullptr)
          ? 0
          : manager().deformable_driver()->num_deformable_bodies();
  const int num_objects = num_rigid_bodies + num_deformable_bodies;
  cache->sap_problem = std::make_unique<SapContactProblem<T>>(
      plant().time_step(), std::move(A), std::move(v_star));
  cache->sap_problem->set_num_objects(num_objects);
  SapContactProblem<T>& problem = *cache->sap_problem;
  // N.B. All contact constraints must be added before any other constraint
  // types. This driver assumes this ordering of the constraints in order to
  // extract contact impulses for reporting contact results.
  // Do not change this order here!
  cache->R_WC = AddContactConstraints(context, &problem);
  AddLimitConstraints(context, problem.v_star(), &problem);

  cache->pd_controller_constraints_start = problem.num_constraints();
  AddPdControllerConstraints(context, &problem);
  cache->num_pd_controller_constraints =
      problem.num_constraints() - cache->pd_controller_constraints_start;

  AddCouplerConstraints(context, &problem);
  AddDistanceConstraints(context, &problem);
  AddBallConstraints(context, &problem);
  AddWeldConstraints(context, &problem);
  AddFixedConstraints(context, &problem);

  // Make a reduced version of the original contact problem using joint locking
  // data.
  const internal::JointLockingCacheData<T>& joint_locking_data =
      manager().EvalJointLockingCache(context);
  const std::vector<int>& locked_indices =
      joint_locking_data.locked_velocity_indices;
  const std::vector<std::vector<int>>& locked_indices_per_tree =
      joint_locking_data.locked_velocity_indices_per_tree;

  // If any tree has at least one known DoF, then we have to consider joint
  // locking. Otherwise use nullptr semantics on the locked problem to indicate
  // no joint locking is present.
  bool has_locked_dofs =
      std::any_of(locked_indices_per_tree.begin(),
                  locked_indices_per_tree.end(), [](const std::vector<int>& v) {
                    return v.size() > 0;
                  });

  if (has_locked_dofs) {
    cache->sap_problem_locked = std::move(problem.MakeReduced(
        locked_indices, locked_indices_per_tree, &cache->mapping));
  } else {
    cache->sap_problem_locked = nullptr;
  }
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
        manager().deformable_driver();
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
    const int v_start = tree_topology().tree_velocities_start_in_v(t);
    const int nv = tree_topology().num_tree_velocities(t);
    values->segment(v_start, nv) += clique_values;
  }
}

template <typename T>
void SapDriver<T>::CalcSapSolverResults(
    const systems::Context<T>& context,
    SapSolverResults<T>* sap_results) const {
  const ContactProblemCache<T>& contact_problem_cache =
      EvalContactProblemCache(context);
  const SapContactProblem<T>& sap_problem = *contact_problem_cache.sap_problem;

  bool has_locked_dofs = (contact_problem_cache.sap_problem_locked != nullptr);

  // We use the velocity stored in the current context as initial guess.
  const VectorX<T>& x0 =
      context.get_discrete_state(manager().multibody_state_index()).value();
  VectorX<T> v0 = x0.bottomRows(this->plant().num_velocities());

  // Eliminate known DoFs.
  if (has_locked_dofs) {
    const auto& unlocked_indices =
        manager().EvalJointLockingCache(context).unlocked_velocity_indices;
    v0 = SelectRows(v0, unlocked_indices);
  }

  if constexpr (std::is_same_v<T, double>) {
    if (manager().deformable_driver() != nullptr) {
      const VectorX<double> deformable_v0 =
          manager().deformable_driver()->EvalParticipatingVelocities(context);
      const int rigid_dofs = v0.size();
      const int deformable_dofs = deformable_v0.size();
      v0.conservativeResize(rigid_dofs + deformable_dofs);
      v0.tail(deformable_dofs) = deformable_v0;
    }
  }

  // Solve the reduced DOF locked problem.
  SapSolver<T> sap;
  sap.set_parameters(sap_parameters_);

  SapSolverStatus status;
  if (has_locked_dofs) {
    SapSolverResults<T> locked_sap_results;
    status = sap.SolveWithGuess(*contact_problem_cache.sap_problem_locked, v0,
                                &locked_sap_results);
    if (status == SapSolverStatus::kSuccess) {
      sap_problem.ExpandContactSolverResults(contact_problem_cache.mapping,
                                             locked_sap_results, sap_results);
    }
  } else {
    status = sap.SolveWithGuess(sap_problem, v0, sap_results);
  }

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
        "  4. Ill-conditioning could be alleviated via SAP's near rigid "
        "     parameter. Refer to "
        "     MultibodyPlant::set_sap_near_rigid_threshold() for details."
        "  5. Some other cause. You may want to use Stack Overflow (#drake "
        "     tag) to request some assistance.",
        context.get_time());
    throw std::runtime_error(msg);
  }
}

template <typename T>
void SapDriver<T>::CalcContactSolverResults(
    const systems::Context<T>& context,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  const SapSolverResults<T>& sap_results = EvalSapSolverResults(context);
  const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs =
      manager().EvalDiscreteContactPairs(context);
  const int num_contacts = contact_pairs.size();
  const ContactProblemCache<T>& contact_problem_cache =
      EvalContactProblemCache(context);
  PackContactSolverResults(context, *contact_problem_cache.sap_problem,
                           num_contacts, sap_results, results);
}

template <typename T>
void SapDriver<T>::CalcDiscreteUpdateMultibodyForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  auto& generalized_forces = forces->mutable_generalized_forces();
  auto& spatial_forces = forces->mutable_body_forces();

  // Current state (previous time step).
  const VectorX<T>& x0 =
      context.get_discrete_state(manager().multibody_state_index()).value();
  const auto v0 = x0.bottomRows(plant().num_velocities());

  // Next time step state.
  const SapSolverResults<T>& sap_results = EvalSapSolverResults(context);
  // Generalized velocities and accelerations.
  const VectorX<T>& v = sap_results.v;
  const VectorX<T> a = (v - v0) / plant().time_step();

  // Include all state dependent forces (not constraints) evaluated at t₀
  // (previous time step as stored in the context).
  manager().CalcNonContactForces(context,
                                 /* include_joint_limit_penalty_forces */ false,
                                 /* include_pd_controlled_input */ false,
                                 forces);

  // SAP evaluates damping terms (joint damping and reflected inertia)
  // implicitly. Therefore we must subtract the explicit term evaluated above
  // and include the implicit term instead.
  const VectorX<T> diagonal_inertia = manager().CalcEffectiveDamping(context);
  generalized_forces -= diagonal_inertia.asDiagonal() * a;

  // Include the contribution from constraints.
  // TODO(amcastro-tri): Consider deformables.
  if (manager().deformable_driver() != nullptr) {
    throw std::logic_error(
        "The computation of MultibodyForces must be updated to include "
        "deformable objects.");
  }

  VectorX<T> constraints_generalized_forces(plant().num_velocities());
  std::vector<SpatialForce<T>> constraint_spatial_forces(plant().num_bodies());
  const VectorX<T>& gamma = sap_results.gamma;

  // N.B. When CompliantContactManager builds the problem, the "about point" for
  // the reporting of multibody forces is defined to be at body origins and
  // expressed in the world frame.
  // Therefore aggregation of forces per-body makes sense in this call.
  const ContactProblemCache<T>& contact_problem_cache =
      EvalContactProblemCache(context);
  contact_problem_cache.sap_problem->CalcConstraintMultibodyForces(
      gamma, &constraints_generalized_forces, &constraint_spatial_forces);
  generalized_forces += constraints_generalized_forces;

  // N.B. The CompliantContactManager indexes constraints objects with body
  // indexes. Therefore using body indices on constraint_spatial_forces is
  // correct. However MultibodyForce uses MobodIndex, see below.
  for (BodyIndex b(0); b < plant().num_bodies(); ++b) {
    // MultibodyForce indexes spatial body forces by MobodIndex.
    const MobodIndex mobod_index = plant().get_body(b).mobod_index();
    spatial_forces[mobod_index] += constraint_spatial_forces[b];
  }
}

template <typename T>
void SapDriver<T>::CalcActuation(const systems::Context<T>& context,
                                 VectorX<T>* actuation) const {
  // By default, models with no controllers feed through the output.
  // PD controlled actuation values are overwritten below with values computed
  // by the SAP solver, which includes these terms implicitly and enforces
  // effort limits.
  *actuation = manager().AssembleActuationInput(context);

  // Add contribution from PD controllers.
  const ContactProblemCache<T>& contact_problem_cache =
      EvalContactProblemCache(context);

  const int start = contact_problem_cache.pd_controller_constraints_start;
  const int num_pd_constraints =
      contact_problem_cache.num_pd_controller_constraints;
  if (num_pd_constraints == 0) return;
  const int end = start + num_pd_constraints - 1;

  const SapSolverResults<T>& sap_results = EvalSapSolverResults(context);
  const VectorX<T>& gamma = sap_results.gamma;
  VectorX<T> tau_pd = VectorX<T>::Zero(plant().num_velocities());
  const SapContactProblem<T>& sap_problem = *contact_problem_cache.sap_problem;
  sap_problem.CalcConstraintGeneralizedForces(gamma, start, end, &tau_pd);

  // Map generalized forces to actuation indexing.
  int constraint_index = start;
  for (JointActuatorIndex actuator_index(0);
       actuator_index < plant().num_actuators(); ++actuator_index) {
    const JointActuator<T>& actuator =
        plant().get_joint_actuator(actuator_index);
    const Joint<T>& joint = actuator.joint();
    // PD constraints are not even added to the problem when joints are locked.
    // PD actuation is considered to be zero and only the input actuation is
    // reported.
    if (actuator.has_controller() && !joint.is_locked(context)) {
      const SapConstraint<T>& c =
          sap_problem.get_constraint(constraint_index++);
      const int dof = joint.velocity_start();

      // We know that PD controllers constraints are one equation each.
      DRAKE_DEMAND(c.num_constraint_equations() == 1);

      // Each actuator defines a single PD controller.
      actuation->coeffRef(actuator_index) = tau_pd(dof);
    }
  }
  // Sanity check consistency with the code that added the constraints,
  // AddPdControllerConstraints().
  DRAKE_DEMAND(constraint_index - 1 == end);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::SapDriver);
