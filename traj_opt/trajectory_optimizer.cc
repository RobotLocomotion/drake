#include "drake/traj_opt/trajectory_optimizer.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/traj_opt/penta_diagonal_solver.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace traj_opt {

using geometry::GeometryId;
using geometry::SignedDistancePair;
using internal::PentaDiagonalFactorization;
using internal::PentaDiagonalFactorizationStatus;
using math::RigidTransform;
using multibody::Body;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::Joint;
using multibody::JointIndex;
using multibody::MultibodyPlant;
using multibody::SpatialForce;
using multibody::SpatialVelocity;
using systems::System;

template <typename T>
TrajectoryOptimizer<T>::TrajectoryOptimizer(const MultibodyPlant<T>* plant,
                                            Context<T>* context,
                                            const ProblemDefinition& prob,
                                            const SolverParameters& params)
    : plant_(plant), context_(context), prob_(prob), params_(params) {
  // Define joint damping coefficients.
  joint_damping_ = VectorX<T>::Zero(plant_->num_velocities());

  for (JointIndex j(0); j < plant_->num_joints(); ++j) {
    const Joint<T>& joint = plant_->get_joint(j);
    const int velocity_start = joint.velocity_start();
    const int nv = joint.num_velocities();
    joint_damping_.segment(velocity_start, nv) = joint.damping_vector();
  }

  if (params_.gradients_method == GradientsMethod::kAutoDiff) {
    throw std::runtime_error(
        "It is not possible to use automatic differentiation when only the "
        "plant is provided. Use the constructor providing the full Diagram.");
  }
}

template <typename T>
TrajectoryOptimizer<T>::TrajectoryOptimizer(
    const Diagram<T>* diagram, const MultibodyPlant<T>* plant,
    const ProblemDefinition& prob,
    const SolverParameters& params)
    : diagram_{diagram}, plant_(plant), prob_(prob), params_(params) {
  // Workaround for when a plant context is not provided.
  // Valid context should be obtained with EvalPlantContext() instead.
  // TODO(amcastro-tri): get rid of this.
  owned_context_ = diagram->CreateDefaultContext();
  context_ = &diagram->GetMutableSubsystemContext(*plant, owned_context_.get());

  // Define joint damping coefficients.
  joint_damping_ = VectorX<T>::Zero(plant_->num_velocities());

  for (JointIndex j(0); j < plant_->num_joints(); ++j) {
    const Joint<T>& joint = plant_->get_joint(j);
    const int velocity_start = joint.velocity_start();
    const int nv = joint.num_velocities();
    joint_damping_.segment(velocity_start, nv) = joint.damping_vector();
  }

  if constexpr (std::is_same_v<T, double>) {
    if (params_.gradients_method == GradientsMethod::kAutoDiff) {
      diagram_ad_ = systems::System<double>::ToAutoDiffXd(*diagram);
      plant_ad_ = dynamic_cast<const MultibodyPlant<AutoDiffXd>*>(
          &diagram_ad_->GetSubsystemByName(plant->get_name()));
      DRAKE_DEMAND(plant_ad_ != nullptr);
      optimizer_ad_ = std::make_unique<TrajectoryOptimizer<AutoDiffXd>>(
          diagram_ad_.get(), plant_ad_, prob, params);
      // TODO: move state's destructor and possible other implementation to the
      // source?
      state_ad_ = std::unique_ptr<TrajectoryOptimizerState<AutoDiffXd>>(
          new TrajectoryOptimizerState<AutoDiffXd>(num_steps(), *diagram_ad_,
                                                   *plant_ad_));
    }
  } else {
    throw std::runtime_error(
        "Analytical gradients not supported for "
        "TrajectoryOptimizer<AutoDiffXd>.");
  }
}

template <typename T>
const T TrajectoryOptimizer<T>::EvalCost(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().cost_up_to_date) {
    state.mutable_cache().cost = CalcCost(state);
    state.mutable_cache().cost_up_to_date = true;
  }
  return state.cache().cost;
}

template <typename T>
T TrajectoryOptimizer<T>::CalcCost(
    const TrajectoryOptimizerState<T>& state) const {
  const std::vector<VectorX<T>>& v = EvalV(state);
  const std::vector<VectorX<T>>& tau = EvalTau(state);
  T cost = CalcCost(state.q(), v, tau, &state.workspace);

  // Add a proximal operator term to the cost, if requested
  if (params_.proximal_operator) {
    const std::vector<VectorX<T>>& q = state.q();
    const std::vector<VectorX<T>>& q_last =
        state.proximal_operator_data().q_last;
    const std::vector<VectorX<T>>& H_diag =
        state.proximal_operator_data().H_diag;
    for (int t = 0; t <= num_steps(); ++t) {
      cost += T(0.5 * params_.rho_proximal * (q[t] - q_last[t]).transpose() *
                H_diag[t].asDiagonal() * (q[t] - q_last[t]));
    }
  }

  return cost;
}

template <typename T>
T TrajectoryOptimizer<T>::CalcCost(
    const std::vector<VectorX<T>>& q, const std::vector<VectorX<T>>& v,
    const std::vector<VectorX<T>>& tau,
    TrajectoryOptimizerWorkspace<T>* workspace) const {
  T cost = 0;
  VectorX<T>& q_err = workspace->q_size_tmp1;
  VectorX<T>& v_err = workspace->v_size_tmp1;

  // Running cost
  for (int t = 0; t < num_steps(); ++t) {
    q_err = q[t] - prob_.q_nom;
    v_err = v[t] - prob_.v_nom;
    cost += T(q_err.transpose() * prob_.Qq * q_err);
    cost += T(v_err.transpose() * prob_.Qv * v_err);
    cost += T(tau[t].transpose() * prob_.R * tau[t]);
  }

  // Scale running cost by dt (so the optimization problem we're solving doesn't
  // change so dramatically when we change the time step).
  cost *= time_step();

  // Terminal cost
  q_err = q[num_steps()] - prob_.q_nom;
  v_err = v[num_steps()] - prob_.v_nom;
  cost += T(q_err.transpose() * prob_.Qf_q * q_err);
  cost += T(v_err.transpose() * prob_.Qf_v * v_err);

  return cost;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcVelocities(const std::vector<VectorX<T>>& q,
                                            std::vector<VectorX<T>>* v) const {
  // x = [x0, x1, ..., xT]
  DRAKE_DEMAND(static_cast<int>(q.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(v->size()) == num_steps() + 1);

  v->at(0) = prob_.v_init;
  for (int t = 1; t <= num_steps(); ++t) {
    v->at(t) = (q[t] - q[t - 1]) / time_step();
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcAccelerations(
    const std::vector<VectorX<T>>& v, std::vector<VectorX<T>>* a) const {
  DRAKE_DEMAND(static_cast<int>(v.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(a->size()) == num_steps());

  for (int t = 0; t < num_steps(); ++t) {
    a->at(t) = (v[t + 1] - v[t]) / time_step();
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamics(
    const std::vector<VectorX<T>>& q, const std::vector<VectorX<T>>& v,
    const std::vector<VectorX<T>>& a,
    TrajectoryOptimizerWorkspace<T>* workspace,
    std::vector<VectorX<T>>* tau) const {
  // Generalized forces aren't defined for the last timestep
  // TODO(vincekurtz): additional checks that q_t, v_t, tau_t are the right size
  // for the plant?
  DRAKE_DEMAND(static_cast<int>(q.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(v.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(a.size()) == num_steps());
  DRAKE_DEMAND(static_cast<int>(tau->size()) == num_steps());

  for (int t = 0; t < num_steps(); ++t) {
    // All dynamics terms are treated implicitly, i.e.,
    // tau[t] = M(q[t+1]) * a[t] - k(q[t+1],v[t+1]) - f_ext[t+1]
    CalcInverseDynamicsSingleTimeStep(q[t + 1], v[t + 1], a[t], workspace,
                                      &tau->at(t));
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsSingleTimeStep(
    const VectorX<T>& q, const VectorX<T>& v, const VectorX<T>& a,
    TrajectoryOptimizerWorkspace<T>* workspace, VectorX<T>* tau) const {
  plant().SetPositions(context_, q);
  plant().SetVelocities(context_, v);
  plant().CalcForceElementsContribution(*context_, &workspace->f_ext);

  // Add in contact force contribution to f_ext
  if (plant().geometry_source_is_registered()) {
    // Only compute contact forces if the plant is connected to a scene graph
    // TODO(vincekurtz): perform this check earlier, and maybe print some
    // warnings to stdout if we're not connected (we do want to be able to run
    // problems w/o contact sometimes)
    CalcContactForceContribution(&workspace->f_ext);
  }

  // Inverse dynamics computes tau = M*a - k(q,v) - f_ext
  *tau = plant().CalcInverseDynamics(*context_, a, workspace->f_ext);
}

template <typename T>
void TrajectoryOptimizer<T>::CalcContactForceContribution(
    MultibodyForces<T>* forces) const {
  using std::abs;
  using std::max;
  using std::pow;
  using std::sqrt;

  // Compliant contact parameters. stiffness_exponent = 3/2 corresponds to Hertz
  // model for spherical contact. stiffness_exponent = 1.0 corresponds to a
  // linear spring force with stiffness F/delta.
  const double F = params_.F;
  const double delta = params_.delta;
  const double stiffness_exponent = params_.stiffness_exponent;

  // (Normal) Dissipation. dissipation_exponent = 1.0 corresponds to the Hunt &
  // Crossley model of dissipation.
  const double dissipation_exponent = params_.dissipation_exponent;
  const double dissipation_velocity = params_.dissipation_velocity;

  // Friction parameters.
  const double vs = params_.stiction_velocity;     // Regularization.
  const double mu = params_.friction_coefficient;  // Coefficient of friction.

  // Get signed distance pairs
  const geometry::QueryObject<T>& query_object =
      plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(*context_);
  const std::vector<SignedDistancePair<T>>& signed_distance_pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints();
  const drake::geometry::SceneGraphInspector<T>& inspector =
      query_object.inspector();

  for (const SignedDistancePair<T>& pair : signed_distance_pairs) {
    // Don't do any contact force computations if we're not in contact
    if (pair.distance < 0) {
      // Normal outwards from A.
      const Vector3<T> nhat = -pair.nhat_BA_W;

      // Get geometry and transformation data for the witness points
      const GeometryId geometryA_id = pair.id_A;
      const GeometryId geometryB_id = pair.id_B;

      const BodyIndex bodyA_index =
          plant().geometry_id_to_body_index().at(geometryA_id);
      const Body<T>& bodyA = plant().get_body(bodyA_index);
      const BodyIndex bodyB_index =
          plant().geometry_id_to_body_index().at(geometryB_id);
      const Body<T>& bodyB = plant().get_body(bodyB_index);

      // Body poses in world.
      const math::RigidTransform<T>& X_WA =
          plant().EvalBodyPoseInWorld(*context_, bodyA);
      const math::RigidTransform<T>& X_WB =
          plant().EvalBodyPoseInWorld(*context_, bodyB);

      // Geometry poses in body frames.
      const math::RigidTransform<T> X_AGa =
          inspector.GetPoseInParent(geometryA_id).template cast<T>();
      const math::RigidTransform<T> X_BGb =
          inspector.GetPoseInParent(geometryB_id).template cast<T>();

      // Position of the witness points in the world frame.
      const auto& p_GaCa_Ga = pair.p_ACa;
      const RigidTransform<T> X_WGa = X_WA * X_AGa;
      const Vector3<T> p_WCa_W = X_WGa * p_GaCa_Ga;
      const auto& p_GbCb_Gb = pair.p_BCb;
      const RigidTransform<T> X_WGb = X_WB * X_BGb;
      const Vector3<T> p_WCb_W = X_WGb * p_GbCb_Gb;

      // We define the (common, unique) contact point C as the midpoint between
      // witness points Ca and Cb.
      const Vector3<T> p_WC = 0.5 * (p_WCa_W + p_WCb_W);

      // Shift vectors.
      const Vector3<T> p_AC_W = p_WC - X_WA.translation();
      const Vector3<T> p_BC_W = p_WC - X_WB.translation();

      // Velocities.
      const SpatialVelocity<T>& V_WA =
          plant().EvalBodySpatialVelocityInWorld(*context_, bodyA);
      const SpatialVelocity<T>& V_WB =
          plant().EvalBodySpatialVelocityInWorld(*context_, bodyB);
      const SpatialVelocity<T> V_WAc = V_WA.Shift(p_AC_W);
      const SpatialVelocity<T> V_WBc = V_WB.Shift(p_BC_W);

      // Relative contact velocity.
      const Vector3<T> v_AcBc_W = V_WBc.translational() - V_WAc.translational();

      // Split into normal and tangential components.
      const T vn = nhat.dot(v_AcBc_W);
      const Vector3<T> vt = v_AcBc_W - vn * nhat;

      // Normal (compliant) component.
      const T sign_vn = vn > 0 ? 1.0 : -1.0;
      const T dissipation_factor = max(
          0.0, 1.0 - pow(abs(vn / dissipation_velocity), dissipation_exponent) *
                         sign_vn);
      const T compliant_fn =
          F * pow(-pair.distance / delta, stiffness_exponent);
      const T fn = compliant_fn * dissipation_factor;

      // Tangential frictional component.
      // N.B. This model is algebraically equivalent to:
      //  ft = -mu*fn*sigmoid(||vt||/vs)*vt/||vt||.
      // with the algebraic sigmoid function defined as sigmoid(x) =
      // x/sqrt(1+x^2). The algebraic simplification is performed to avoid
      // division by zero when vt = 0 (or lost of precision when close to zero).
      const Vector3<T> that_regularized =
          -vt / sqrt(vs * vs + vt.squaredNorm());
      const Vector3<T> ft_BC_W = that_regularized * mu * fn;

      // Total contact force on B at C, expressed in W.
      const Vector3<T> f_BC_W = nhat * fn + ft_BC_W;

      // Spatial contact forces on bodies A and B.
      const SpatialForce<T> F_BC_W(Vector3<T>::Zero(), f_BC_W);
      const SpatialForce<T> F_BBo_W = F_BC_W.Shift(-p_BC_W);

      const SpatialForce<T> F_AC_W(Vector3<T>::Zero(), -f_BC_W);
      const SpatialForce<T> F_AAo_W = F_AC_W.Shift(-p_AC_W);

      // Add the forces into the given MultibodyForces
      forces->mutable_body_forces()[bodyA.node_index()] += F_AAo_W;
      forces->mutable_body_forces()[bodyB.node_index()] += F_BBo_W;
    }
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcSdfData(
    const TrajectoryOptimizerState<T>& state,
    typename TrajectoryOptimizerCache<T>::SdfData* sdf_data) const {
  sdf_data->sdf_pairs.resize(num_steps());
  for (int t = 0; t < num_steps(); ++t) {
    const Context<T>& context = EvalPlantContext(state, t);
    const geometry::QueryObject<T>& query_object =
        plant()
            .get_geometry_query_input_port()
            .template Eval<geometry::QueryObject<T>>(context);
    sdf_data->sdf_pairs[t] =
        query_object.ComputeSignedDistancePairwiseClosestPoints();
  }
  sdf_data->up_to_date = true;
}

template <typename T>
const std::vector<geometry::SignedDistancePair<T>>&
TrajectoryOptimizer<T>::EvalSignedDistancePairs(
    const TrajectoryOptimizerState<T>& state, int t) const {
  DRAKE_DEMAND(0 <= t && t < num_steps());
  if (!state.cache().sdf_data.up_to_date) {
    CalcSdfData(state, &state.mutable_cache().sdf_data);
  }
  return state.cache().sdf_data.sdf_pairs[t];
}

template <typename T>
void TrajectoryOptimizer<T>::CalcContactJacobian(
    const Context<T>& context,
    const std::vector<geometry::SignedDistancePair<T>>& sdf_pairs,
    MatrixX<T>* J, std::vector<math::RotationMatrix<T>>* R_WC,
    std::vector<std::pair<BodyIndex, BodyIndex>>* body_pairs) const {
  const geometry::QueryObject<T>& query_object =
      plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const drake::geometry::SceneGraphInspector<T>& inspector =
      query_object.inspector();

  // TODO(amcastro-tri): consider moving into workspace to avoid heap
  // allocating.
  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  const int nc = sdf_pairs.size();
  J->resize(3 * nc, nv);
  R_WC->resize(nc);
  body_pairs->resize(nc);

  int ic = 0;
  const Frame<T>& frame_W = plant().world_frame();
  for (const SignedDistancePair<T>& pair : sdf_pairs) {
    // Normal outwards from A.
    const Vector3<T> nhat_W = -pair.nhat_BA_W;

    // Get geometry and transformation data for the witness points
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const BodyIndex bodyA_index =
        plant().geometry_id_to_body_index().at(geometryA_id);
    const Body<T>& bodyA = plant().get_body(bodyA_index);
    const BodyIndex bodyB_index =
        plant().geometry_id_to_body_index().at(geometryB_id);
    const Body<T>& bodyB = plant().get_body(bodyB_index);

    body_pairs->at(ic) = std::make_pair(bodyA_index, bodyB_index);

    // Body poses in world.
    const math::RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, bodyA);
    const math::RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, bodyB);

    // Geometry poses in body frames.
    const math::RigidTransform<T> X_AGa =
        inspector.GetPoseInParent(geometryA_id).template cast<T>();
    const math::RigidTransform<T> X_BGb =
        inspector.GetPoseInParent(geometryB_id).template cast<T>();

    // Position of the witness points in the world frame.
    const auto& p_GaCa_Ga = pair.p_ACa;
    const RigidTransform<T> X_WGa = X_WA * X_AGa;
    const Vector3<T> p_WCa_W = X_WGa * p_GaCa_Ga;
    const auto& p_GbCb_Gb = pair.p_BCb;
    const RigidTransform<T> X_WGb = X_WB * X_BGb;
    const Vector3<T> p_WCb_W = X_WGb * p_GbCb_Gb;

    // We define the (common, unique) contact point C as the midpoint between
    // witness points Ca and Cb.
    const Vector3<T> p_WC = 0.5 * (p_WCa_W + p_WCb_W);

    // Since v_AcBc_W = v_WBc - v_WAc the relative velocity Jacobian will be:
    //   J_AcBc_W = Jv_WBc_W - Jv_WAc_W.
    // That is the relative velocity at C is v_AcBc_W = J_AcBc_W * v.
    const Vector3<T> p_AoC_A = X_WA.inverse() * p_WC;
    plant().CalcJacobianTranslationalVelocity(
        context, multibody::JacobianWrtVariable::kV, bodyA.body_frame(),
        p_AoC_A, frame_W, frame_W, &Jv_WAc_W);
    const Vector3<T> p_BoC_B = X_WB.inverse() * p_WC;
    plant().CalcJacobianTranslationalVelocity(
        context, multibody::JacobianWrtVariable::kV, bodyB.body_frame(),
        p_BoC_B, frame_W, frame_W, &Jv_WBc_W);

    // Define a contact frame C at the contact point such that the z-axis Cz
    // equals nhat_W. The tangent vectors are arbitrary, with the only
    // requirement being that they form a valid right handed basis with nhat_W.
    R_WC->at(ic) = math::RotationMatrix<T>::MakeFromOneVector(nhat_W, 2);

    // Contact Jacobian J_AcBc_C, expressed in the contact frame C.
    // That is, vc = J * v stores the contact velocities expressed in the
    // contact frame C. Similarly for contact forces, they are expressed in this
    // same frame C.
    J->middleRows(3 * ic, 3) =
        R_WC->at(ic).matrix().transpose() * (Jv_WBc_W - Jv_WAc_W);

    ++ic;
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcContactJacobianData(
    const TrajectoryOptimizerState<T>& state,
    typename TrajectoryOptimizerCache<T>::ContactJacobianData*
        contact_jacobian_data) const {
  // Resize contact data accordingly.
  // We resize to include all pairs, even for positive distances for which the
  // contact forces will be zero.
  contact_jacobian_data->J.resize(num_steps());
  contact_jacobian_data->R_WC.resize(num_steps());
  contact_jacobian_data->body_pairs.resize(num_steps());

  for (int t = 0; t < num_steps(); ++t) {
    const Context<T>& context = EvalPlantContext(state, t);
    const std::vector<geometry::SignedDistancePair<T>>& sdf_pairs =
        EvalSignedDistancePairs(state, t);
    CalcContactJacobian(context, sdf_pairs, &contact_jacobian_data->J[t],
                        &contact_jacobian_data->R_WC[t],
                        &contact_jacobian_data->body_pairs[t]);
  }
}

template <typename T>
const typename TrajectoryOptimizerCache<T>::ContactJacobianData&
TrajectoryOptimizer<T>::EvalContactJacobianData(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().contact_jacobian_data.up_to_date) {
    CalcContactJacobianData(state,
                            &state.mutable_cache().contact_jacobian_data);
  }
  return state.cache().contact_jacobian_data;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartials(
    const TrajectoryOptimizerState<T>& state,
    TrajectoryOptimizerWorkspace<T>* workspace,
    InverseDynamicsPartials<T>* id_partials) const {
  // TODO(vincekurtz): use a solver flag to choose between finite differences
  // and an analytical approximation
  switch (params_.gradients_method) {
    case GradientsMethod::kForwardDifferences: {
      CalcInverseDynamicsPartialsFiniteDiff(state, workspace, id_partials);
      break;
    }
    case GradientsMethod::kAutoDiff: {
      if constexpr (std::is_same_v<T, double>) {
        CalcInverseDynamicsPartialsAutoDiff(state, id_partials);
      } else {
        throw std::runtime_error(
            "Analytical gradients not supported for "
            "TrajectoryOptimizer<AutoDiffXd>.");
      }
      break;
    }
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartialsFiniteDiff(
    const TrajectoryOptimizerState<T>& state,
    TrajectoryOptimizerWorkspace<T>* workspace,
    InverseDynamicsPartials<T>* id_partials) const {
  using std::abs;
  using std::max;
  // Check that id_partials has been allocated correctly.
  DRAKE_DEMAND(id_partials->size() == num_steps());

  // Get the trajectory data
  const std::vector<VectorX<T>>& q = state.q();
  const std::vector<VectorX<T>>& v = EvalV(state);
  const std::vector<VectorX<T>>& a = EvalA(state);
  const std::vector<VectorX<T>>& tau = EvalTau(state);

  // Get references to the partials that we'll be setting
  std::vector<MatrixX<T>>& dtau_dqm = id_partials->dtau_dqm;
  std::vector<MatrixX<T>>& dtau_dqt = id_partials->dtau_dqt;
  std::vector<MatrixX<T>>& dtau_dqp = id_partials->dtau_dqp;

  // Get references to perturbed versions of q, v, tau, and a, at (t-1, t, t).
  // These are all of the quantities that change when we perturb q_t.
  VectorX<T>& q_eps_t = workspace->q_size_tmp1;
  VectorX<T>& v_eps_t = workspace->v_size_tmp1;
  VectorX<T>& v_eps_tp = workspace->v_size_tmp2;
  VectorX<T>& a_eps_tm = workspace->a_size_tmp1;
  VectorX<T>& a_eps_t = workspace->a_size_tmp2;
  VectorX<T>& a_eps_tp = workspace->a_size_tmp3;
  VectorX<T>& tau_eps_tm = workspace->tau_size_tmp1;
  VectorX<T>& tau_eps_t = workspace->tau_size_tmp2;
  VectorX<T>& tau_eps_tp = workspace->tau_size_tmp3;

  // Store small perturbations
  const double eps = sqrt(std::numeric_limits<double>::epsilon());
  T dq_i;
  T dv_i;
  T da_i;
  for (int t = 0; t <= num_steps(); ++t) {
    // N.B. A perturbation of qt propagates to tau[t-1], tau[t] and tau[t+1].
    // Therefore we compute one column of grad_tau at a time. That is, once the
    // loop on position indices i is over, we effectively computed the t-th
    // column of grad_tau.

    // Set perturbed versions of variables
    q_eps_t = q[t];
    v_eps_t = v[t];
    if (t < num_steps()) {
      // v[num_steps + 1] is not defined
      v_eps_tp = v[t + 1];
      // a[num_steps] is not defined
      a_eps_t = a[t];
    }
    if (t < num_steps() - 1) {
      // a[num_steps + 1] is not defined
      a_eps_tp = a[t + 1];
    }
    if (t > 0) {
      // a[-1] is undefined
      a_eps_tm = a[t - 1];
    }

    for (int i = 0; i < plant().num_positions(); ++i) {
      // Determine perturbation sizes to avoid losing precision to floating
      // point error
      dq_i = eps * max(1.0, abs(q_eps_t(i)));

      // Make dqt_i exactly representable to minimize floating point error
      const T temp = q_eps_t(i) + dq_i;
      dq_i = temp - q_eps_t(i);

      dv_i = dq_i / time_step();
      da_i = dv_i / time_step();

      // Perturb q_t[i], v_t[i], and a_t[i]
      // TODO(vincekurtz): add N(q)+ factor to consider quaternion DoFs.
      q_eps_t(i) += dq_i;

      if (t == 0) {
        // v[0] is constant
        a_eps_t(i) -= 1.0 * da_i;
      } else {
        v_eps_t(i) += dv_i;
        a_eps_tm(i) += da_i;
        a_eps_t(i) -= 2.0 * da_i;
      }
      v_eps_tp(i) -= dv_i;
      a_eps_tp(i) += da_i;

      // Compute perturbed tau(q) and calculate the nonzero entries of dtau/dq
      // via finite differencing
      if (t > 0) {
        // tau[t-1] = ID(q[t], v[t], a[t-1])
        CalcInverseDynamicsSingleTimeStep(q_eps_t, v_eps_t, a_eps_tm, workspace,
                                          &tau_eps_tm);
        dtau_dqp[t - 1].col(i) = (tau_eps_tm - tau[t - 1]) / dq_i;
      }
      if (t < num_steps()) {
        // tau[t] = ID(q[t+1], v[t+1], a[t])
        CalcInverseDynamicsSingleTimeStep(q[t + 1], v_eps_tp, a_eps_t,
                                          workspace, &tau_eps_t);
        dtau_dqt[t].col(i) = (tau_eps_t - tau[t]) / dq_i;
      }
      if (t < num_steps() - 1) {
        // tau[t+1] = ID(q[t+2], v[t+2], a[t+1])
        CalcInverseDynamicsSingleTimeStep(q[t + 2], v[t + 2], a_eps_tp,
                                          workspace, &tau_eps_tp);
        dtau_dqm[t + 1].col(i) = (tau_eps_tp - tau[t + 1]) / dq_i;
      }

      // Unperturb q_t[i], v_t[i], and a_t[i]
      q_eps_t(i) = q[t](i);
      v_eps_t(i) = v[t](i);
      if (t < num_steps()) {
        v_eps_tp(i) = v[t + 1](i);
        a_eps_t(i) = a[t](i);
      }
      if (t < num_steps() - 1) {
        a_eps_tp(i) = a[t + 1](i);
      }
      if (t > 0) {
        a_eps_tm(i) = a[t - 1](i);
      }
    }
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartialsAutoDiff(
    const TrajectoryOptimizerState<double>& state,
    InverseDynamicsPartials<double>* id_partials) const {
  DRAKE_DEMAND(id_partials->size() == num_steps());

  // Get references to the partials that we'll be setting
  std::vector<MatrixX<T>>& dtau_dqm = id_partials->dtau_dqm;
  std::vector<MatrixX<T>>& dtau_dqt = id_partials->dtau_dqt;
  std::vector<MatrixX<T>>& dtau_dqp = id_partials->dtau_dqp;  

  // Heap allocations.
  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps() + 1);
  VectorX<AutoDiffXd> tau_ad(plant().num_velocities());
  TrajectoryOptimizerWorkspace<AutoDiffXd> workspace_ad(num_steps(),
                                                        *plant_ad_);

  // Initialize q_ad. First with no derivatives, as a constant.
  const std::vector<VectorX<double>>& q = state.q();
  for (int t = 0; t <= num_steps(); ++t) {
    q_ad[t] = q[t];
  }

  for (int t = 0; t <= num_steps(); ++t) {
    // Set derivatives with respect to q[t].
    q_ad[t] = math::InitializeAutoDiff(q[t], q[t].size());
    state_ad_->set_q(q_ad);

    // q[t] will propagate directly to v[t], v[t+1], a[t-1], a[t] and a[t+1].
    // Therefore we need to re-evaluate them.
    const std::vector<VectorX<AutoDiffXd>>& v_ad =
        optimizer_ad_->EvalV(*state_ad_);
    const std::vector<VectorX<AutoDiffXd>>& a_ad =
        optimizer_ad_->EvalA(*state_ad_);

    // All dynamics terms are treated implicitly, i.e.,
    // tau[t] = M(q[t+1]) * a[t] - k(q[t+1],v[t+1]) - f_ext[t+1]

    // TODO: get rid of context_ in the implementation of
    // CalcInverseDynamicsSingleTimeStep().

    // dtau_dqt[t].
    plant_ad_->SetPositions(context_ad_, q_ad[t + 1]);
    plant_ad_->SetVelocities(context_ad_, v_ad[t + 1]);
    CalcInverseDynamicsSingleTimeStep(*context_ad_, a_ad[t], &workspace_ad,
                                      &tau_ad);
    dtau_dqt[t] = math::ExtractGradient(tau_ad);

    // dtau_dqt[t+1].
    plant_ad_->SetPositions(context_ad_, q_ad[t + 2]);
    plant_ad_->SetVelocities(context_ad_, v_ad[t + 2]);
    CalcInverseDynamicsSingleTimeStep(*context_ad_, a_ad[t + 1], &workspace_ad,
                                      &tau_ad);
    dtau_dqm[t + 1] = math::ExtractGradient(tau_ad);

    // dtau_dqt[t-1].
    plant_ad_->SetPositions(context_ad_, q_ad[t]);
    plant_ad_->SetVelocities(context_ad_, v_ad[t]);
    CalcInverseDynamicsSingleTimeStep(*context_ad_, a_ad[t - 1], &workspace_ad,
                                      &tau_ad);
    dtau_dqp[t - 1] = math::ExtractGradient(tau_ad);

    // Unset derivatives.
    q_ad[t].derivatives().setZero();
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcVelocityPartials(
    const std::vector<VectorX<T>>&, VelocityPartials<T>* v_partials) const {
  if (plant().num_velocities() != plant().num_positions()) {
    throw std::runtime_error("Quaternion DoFs not yet supported");
  } else {
    const int nq = plant().num_positions();
    for (int t = 0; t <= num_steps(); ++t) {
      v_partials->dvt_dqt[t] = 1 / time_step() * MatrixXd::Identity(nq, nq);
      if (t > 0) {
        v_partials->dvt_dqm[t] = -1 / time_step() * MatrixXd::Identity(nq, nq);
      }
    }
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcGradientFiniteDiff(
    const TrajectoryOptimizerState<T>& state, EigenPtr<VectorX<T>> g) const {
  using std::abs;
  using std::max;

  // Perturbed versions of q
  std::vector<VectorX<T>>& q_plus = state.workspace.q_sequence_tmp1;
  std::vector<VectorX<T>>& q_minus = state.workspace.q_sequence_tmp2;
  q_plus = state.q();
  q_minus = state.q();

  // non-constant copy of state that we can perturb
  TrajectoryOptimizerState<T> state_eps = CreateState();

  // Set first block of g (derivatives w.r.t. q_0) to zero, since q0 = q_init
  // are constant.
  g->topRows(plant().num_positions()).setZero();

  // Iterate through rows of g using finite differences
  const double eps = cbrt(std::numeric_limits<double>::epsilon());
  T dqt_i;
  int j = plant().num_positions();
  for (int t = 1; t <= num_steps(); ++t) {
    for (int i = 0; i < plant().num_positions(); ++i) {
      // Set finite difference step size
      dqt_i = eps * max(1.0, abs(state.q()[t](i)));
      q_plus[t](i) += dqt_i;
      q_minus[t](i) -= dqt_i;

      // Set g_j = using central differences
      state_eps.set_q(q_plus);
      T L_plus = CalcCost(state_eps);
      state_eps.set_q(q_minus);
      T L_minus = CalcCost(state_eps);
      (*g)(j) = (L_plus - L_minus) / (2 * dqt_i);

      // reset our perturbed Q and move to the next row of g.
      q_plus[t](i) = state.q()[t](i);
      q_minus[t](i) = state.q()[t](i);
      ++j;
    }
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcGradient(
    const TrajectoryOptimizerState<T>& state, EigenPtr<VectorX<T>> g) const {
  const double dt = time_step();
  const int nq = plant().num_positions();
  TrajectoryOptimizerWorkspace<T>* workspace = &state.workspace;

  const std::vector<VectorX<T>>& q = state.q();
  const std::vector<VectorX<T>>& v = EvalV(state);
  const std::vector<VectorX<T>>& tau = EvalTau(state);

  const VelocityPartials<T>& v_partials = EvalVelocityPartials(state);
  const InverseDynamicsPartials<T>& id_partials =
      EvalInverseDynamicsPartials(state);
  const std::vector<MatrixX<T>>& dvt_dqt = v_partials.dvt_dqt;
  const std::vector<MatrixX<T>>& dvt_dqm = v_partials.dvt_dqm;
  const std::vector<MatrixX<T>>& dtau_dqp = id_partials.dtau_dqp;
  const std::vector<MatrixX<T>>& dtau_dqt = id_partials.dtau_dqt;
  const std::vector<MatrixX<T>>& dtau_dqm = id_partials.dtau_dqm;

  // Set first block of g (derivatives w.r.t. q_0) to zero, since q0 = q_init
  // are constant.
  g->topRows(plant().num_positions()).setZero();

  // Scratch variables for storing intermediate cost terms
  VectorX<T>& qt_term = workspace->q_size_tmp1;
  VectorX<T>& vt_term = workspace->v_size_tmp1;
  VectorX<T>& vp_term = workspace->v_size_tmp2;
  VectorX<T>& taum_term = workspace->tau_size_tmp1;
  VectorX<T>& taut_term = workspace->tau_size_tmp2;
  VectorX<T>& taup_term = workspace->tau_size_tmp3;

  for (int t = 1; t < num_steps(); ++t) {
    // Contribution from position cost
    qt_term = (q[t] - prob_.q_nom).transpose() * 2 * prob_.Qq * dt;

    // Contribution from velocity cost
    vt_term = (v[t] - prob_.v_nom).transpose() * 2 * prob_.Qv * dt * dvt_dqt[t];
    if (t == num_steps() - 1) {
      // The terminal cost needs to be handled differently
      vp_term = (v[t + 1] - prob_.v_nom).transpose() * 2 * prob_.Qf_v *
                dvt_dqm[t + 1];
    } else {
      vp_term = (v[t + 1] - prob_.v_nom).transpose() * 2 * prob_.Qv * dt *
                dvt_dqm[t + 1];
    }

    // Contribution from control cost
    taum_term = tau[t - 1].transpose() * 2 * prob_.R * dt * dtau_dqp[t - 1];
    taut_term = tau[t].transpose() * 2 * prob_.R * dt * dtau_dqt[t];
    if (t == num_steps() - 1) {
      // There is no constrol input at the final timestep
      taup_term.setZero(nq);
    } else {
      taup_term = tau[t + 1].transpose() * 2 * prob_.R * dt * dtau_dqm[t + 1];
    }

    // Put it all together to get the gradient w.r.t q[t]
    g->segment(t * nq, nq) =
        qt_term + vt_term + vp_term + taum_term + taut_term + taup_term;
  }

  // Last step is different, because there is terminal cost and v[t+1] doesn't
  // exist
  taum_term = tau[num_steps() - 1].transpose() * 2 * prob_.R * dt *
              dtau_dqp[num_steps() - 1];
  qt_term = (q[num_steps()] - prob_.q_nom).transpose() * 2 * prob_.Qf_q;
  vt_term = (v[num_steps()] - prob_.v_nom).transpose() * 2 * prob_.Qf_v *
            dvt_dqt[num_steps()];
  g->tail(nq) = qt_term + vt_term + taum_term;

  // Add proximal operator term to the gradient, if requested
  if (params_.proximal_operator) {
    const std::vector<VectorX<T>>& q_last =
        state.proximal_operator_data().q_last;
    const std::vector<VectorX<T>>& H_diag =
        state.proximal_operator_data().H_diag;
    for (int t = 0; t <= num_steps(); ++t) {
      g->segment(t * nq, nq) +=
          params_.rho_proximal * H_diag[t].asDiagonal() * (q[t] - q_last[t]);
    }
  }
}

template <typename T>
const VectorX<T>& TrajectoryOptimizer<T>::EvalGradient(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().gradient_up_to_date) {
    CalcGradient(state, &state.mutable_cache().gradient);
  }
  return state.cache().gradient;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcHessian(
    const TrajectoryOptimizerState<T>& state, PentaDiagonalMatrix<T>* H) const {
  DRAKE_DEMAND(H->is_symmetric());
  DRAKE_DEMAND(H->block_rows() == num_steps() + 1);
  DRAKE_DEMAND(H->block_size() == plant().num_positions());

  // Some convienient aliases
  const double dt = time_step();
  const MatrixX<T> Qq = 2 * prob_.Qq * dt;
  const MatrixX<T> Qv = 2 * prob_.Qv * dt;
  const MatrixX<T> R = 2 * prob_.R * dt;
  const MatrixX<T> Qf_q = 2 * prob_.Qf_q;
  const MatrixX<T> Qf_v = 2 * prob_.Qf_v;

  const VelocityPartials<T>& v_partials = EvalVelocityPartials(state);
  const InverseDynamicsPartials<T>& id_partials =
      EvalInverseDynamicsPartials(state);
  const std::vector<MatrixX<T>>& dvt_dqt = v_partials.dvt_dqt;
  const std::vector<MatrixX<T>>& dvt_dqm = v_partials.dvt_dqm;
  const std::vector<MatrixX<T>>& dtau_dqp = id_partials.dtau_dqp;
  const std::vector<MatrixX<T>>& dtau_dqt = id_partials.dtau_dqt;
  const std::vector<MatrixX<T>>& dtau_dqm = id_partials.dtau_dqm;

  // Get mutable references to the non-zero bands of the Hessian
  std::vector<MatrixX<T>>& A = H->mutable_A();  // 2 rows below diagonal
  std::vector<MatrixX<T>>& B = H->mutable_B();  // 1 row below diagonal
  std::vector<MatrixX<T>>& C = H->mutable_C();  // diagonal

  // Fill in the non-zero blocks
  C[0].setIdentity();  // Initial condition q0 fixed at t=0
  for (int t = 1; t < num_steps(); ++t) {
    // dg_t/dq_t
    MatrixX<T>& dgt_dqt = C[t];
    dgt_dqt = Qq;
    dgt_dqt += dvt_dqt[t].transpose() * Qv * dvt_dqt[t];
    dgt_dqt += dtau_dqp[t - 1].transpose() * R * dtau_dqp[t - 1];
    dgt_dqt += dtau_dqt[t].transpose() * R * dtau_dqt[t];
    if (t < num_steps() - 1) {
      dgt_dqt += dtau_dqm[t + 1].transpose() * R * dtau_dqm[t + 1];
      dgt_dqt += dvt_dqm[t + 1].transpose() * Qv * dvt_dqm[t + 1];
    } else {
      dgt_dqt += dvt_dqm[t + 1].transpose() * Qf_v * dvt_dqm[t + 1];
    }

    // dg_t/dq_{t+1}
    MatrixX<T>& dgt_dqp = B[t + 1];
    dgt_dqp = dtau_dqp[t].transpose() * R * dtau_dqt[t];
    if (t < num_steps() - 1) {
      dgt_dqp += dtau_dqt[t + 1].transpose() * R * dtau_dqm[t + 1];
      dgt_dqp += dvt_dqt[t + 1].transpose() * Qv * dvt_dqm[t + 1];
    } else {
      dgt_dqp += dvt_dqt[t + 1].transpose() * Qf_v * dvt_dqm[t + 1];
    }

    // dg_t/dq_{t+2}
    if (t < num_steps() - 1) {
      MatrixX<T>& dgt_dqpp = A[t + 2];
      dgt_dqpp = dtau_dqp[t + 1].transpose() * R * dtau_dqm[t + 1];
    }
  }

  // dg_t/dq_t for the final timestep
  MatrixX<T>& dgT_dqT = C[num_steps()];
  dgT_dqT = Qf_q;
  dgT_dqT += dvt_dqt[num_steps()].transpose() * Qf_v * dvt_dqt[num_steps()];
  dgT_dqT +=
      dtau_dqp[num_steps() - 1].transpose() * R * dtau_dqp[num_steps() - 1];

  // Add proximal operator terms to the Hessian, if requested
  if (params_.proximal_operator) {
    for (int t = 0; t <= num_steps(); ++t) {
      C[t] += params_.rho_proximal *
              state.proximal_operator_data().H_diag[t].asDiagonal();
    }
  }

  // Copy lower triangular part to upper triangular part
  H->MakeSymmetric();
}

template <typename T>
const PentaDiagonalMatrix<T>& TrajectoryOptimizer<T>::EvalHessian(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().hessian_up_to_date) {
    CalcHessian(state, &state.mutable_cache().hessian);
  }
  return state.cache().hessian;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcCacheTrajectoryData(
    const TrajectoryOptimizerState<T>& state) const {
  TrajectoryOptimizerCache<T>& cache = state.mutable_cache();
  TrajectoryOptimizerWorkspace<T>& workspace = state.workspace;

  // Some aliases for things that we'll set
  std::vector<VectorX<T>>& v = cache.trajectory_data.v;
  std::vector<VectorX<T>>& a = cache.trajectory_data.a;
  std::vector<VectorX<T>>& tau = cache.trajectory_data.tau;

  // The generalized positions that everything is computed from
  const std::vector<VectorX<T>>& q = state.q();

  // Compute corresponding generalized velocities
  CalcVelocities(q, &v);

  // Compute corresponding generalized accelerations
  CalcAccelerations(v, &a);

  // Compute corresponding generalized torques
  CalcInverseDynamics(q, v, a, &workspace, &tau);

  // Set cache invalidation flag
  cache.trajectory_data.up_to_date = true;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcContextCache(
    const TrajectoryOptimizerState<T>& state,
    typename TrajectoryOptimizerCache<T>::ContextCache* cache) const {
  if (diagram_ == nullptr) {
    throw std::runtime_error(
        "No Diagram was provided at construction of the TrajectoryOptimizer. "
        "Use the constructor that takes a Diagram to enable the caching of "
        "contexts.");
  }
  const std::vector<VectorX<T>>& q = state.q();
  const std::vector<VectorX<T>>& v = EvalV(state);
  auto& plant_contexts = cache->plant_contexts;
  for (int t = 0; t <= num_steps(); ++t) {
    plant().SetPositions(plant_contexts[t], q[t]);
    plant().SetVelocities(plant_contexts[t], v[t]);
  }
  cache->up_to_date = true;
}

template <typename T>
const Context<T>& TrajectoryOptimizer<T>::EvalPlantContext(
    const TrajectoryOptimizerState<T>& state, int t) const {
  if (!state.cache().context_cache->up_to_date) {
    CalcContextCache(state, state.mutable_cache().context_cache.get());
  }
  return *state.cache().context_cache->plant_contexts[t];
}

template <typename T>
const std::vector<VectorX<T>>& TrajectoryOptimizer<T>::EvalV(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().trajectory_data.up_to_date) CalcCacheTrajectoryData(state);
  return state.cache().trajectory_data.v;
}

template <typename T>
const std::vector<VectorX<T>>& TrajectoryOptimizer<T>::EvalA(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().trajectory_data.up_to_date) CalcCacheTrajectoryData(state);
  return state.cache().trajectory_data.a;
}

template <typename T>
const std::vector<VectorX<T>>& TrajectoryOptimizer<T>::EvalTau(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().trajectory_data.up_to_date) CalcCacheTrajectoryData(state);
  return state.cache().trajectory_data.tau;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcCacheDerivativesData(
    const TrajectoryOptimizerState<T>& state) const {
  TrajectoryOptimizerCache<T>& cache = state.mutable_cache();
  TrajectoryOptimizerWorkspace<T>& workspace = state.workspace;    

  // Some aliases
  InverseDynamicsPartials<T>& id_partials = cache.derivatives_data.id_partials;
  VelocityPartials<T>& v_partials = cache.derivatives_data.v_partials;

  // Compute partial derivatives of inverse dynamics d(tau)/d(q)
  CalcInverseDynamicsPartials(state, &workspace, &id_partials);

  // Compute partial derivatives of velocities d(v)/d(q)
  const std::vector<VectorX<T>>& q = state.q();
  CalcVelocityPartials(q, &v_partials);

  // Set cache invalidation flag
  cache.derivatives_data.up_to_date = true;
}

template <typename T>
const VelocityPartials<T>& TrajectoryOptimizer<T>::EvalVelocityPartials(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().derivatives_data.up_to_date)
    CalcCacheDerivativesData(state);
  return state.cache().derivatives_data.v_partials;
}

template <typename T>
const InverseDynamicsPartials<T>&
TrajectoryOptimizer<T>::EvalInverseDynamicsPartials(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().derivatives_data.up_to_date)
    CalcCacheDerivativesData(state);
  return state.cache().derivatives_data.id_partials;
}

template <typename T>
void TrajectoryOptimizer<T>::SaveLinePlotDataFirstVariable(
    TrajectoryOptimizerState<T>* scratch_state) const {
  std::ofstream data_file;
  data_file.open("lineplot_data.csv");
  data_file << "q, L, g, H\n";  // header

  // Establish sample points
  const double q_min = params_.lineplot_q_min;
  const double q_max = params_.lineplot_q_max;
  const double num_samples = 10000;
  const double dq = (q_max - q_min) / num_samples;

  const int nq = plant().num_positions();

  // Make a mutable copy of the decision variables
  std::vector<VectorX<T>> q = scratch_state->q();
  double qi = q_min;
  for (int i = 0; i < num_samples; ++i) {
    // Set the decision variables q
    q[1](0) = qi;
    scratch_state->set_q(q);

    // Compute cost, gradient, and Hessian for the first decision variable
    const T& L = EvalCost(*scratch_state);
    const T& g = EvalGradient(*scratch_state)[nq];
    const T& H = EvalHessian(*scratch_state).C()[1](0, 0);

    // Write to the file
    data_file << fmt::format("{}, {}, {}, {}\n", qi, L, g, H);

    // Move to the next value of q
    qi += dq;
  }
}

template <typename T>
void TrajectoryOptimizer<T>::SetupIterationDataFile() const {
  std::ofstream data_file;
  data_file.open("iteration_data.csv");
  data_file << "iter, q, cost, Delta, rho, dq\n";
  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SaveIterationData(
    const int iter, const double Delta, const double rho, const double dq,
    const TrajectoryOptimizerState<T>& state) const {
  std::ofstream data_file;
  data_file.open("iteration_data.csv", std::ios_base::app);

  const T& q = state.q()[1](0);  // assuming 1-DoF and 1 timestep
  const T& cost = EvalCost(state);

  data_file << fmt::format("{}, {}, {}, {}, {}, {}\n", iter, q, cost, Delta,
                           rho, dq);

  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SaveContourPlotDataFirstTwoVariables(
    TrajectoryOptimizerState<T>* scratch_state) const {
  using std::sqrt;
  std::ofstream data_file;
  data_file.open("contour_data.csv");
  data_file
      << "q1, q2, L, g1, g2, H11, H12, H21, H22, g_norm, H_norm\n";  // header

  // Establish sample points
  const double q1_min = params_.contour_q1_min;
  const double q1_max = params_.contour_q1_max;
  const double q2_min = params_.contour_q2_min;
  const double q2_max = params_.contour_q2_max;
  const int nq1 = 150;
  const int nq2 = 150;
  const double dq1 = (q1_max - q1_min) / nq1;
  const double dq2 = (q2_max - q2_min) / nq2;

  T cost;
  std::vector<VectorX<T>> q = scratch_state->q();
  double q1 = q1_min;
  for (int i = 0; i < nq1; ++i) {
    double q2 = q2_min;
    for (int j = 0; j < nq2; ++j) {
      // Update q
      q[1](0) = q1;
      q[1](1) = q2;

      // Compute L(q)
      scratch_state->set_q(q);
      cost = EvalCost(*scratch_state);

      const MatrixX<T> H = EvalHessian(*scratch_state).MakeDense();
      const VectorX<T> g = EvalGradient(*scratch_state);

      // Write to the file
      data_file << fmt::format("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n",
                               q1, q2, cost, g(2), g(3), H(2, 2), H(2, 3),
                               H(3, 2), H(3, 3), g.norm(),
                               H.block(2, 2, 2, 2).norm());

      q2 += dq2;
    }
    q1 += dq1;
  }

  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SetupQuadraticDataFile() const {
  std::ofstream data_file;
  data_file.open("quadratic_data.csv");
  data_file << "iter, q1, q2, dq1, dq2, Delta, cost , g1, g2, H11, H12, H21, "
               "H22, g_norm, H_norm\n";
  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SaveQuadraticDataFirstTwoVariables(
    const int iter, const double Delta, const VectorX<T>& dq,
    const TrajectoryOptimizerState<T>& state) const {
  std::ofstream data_file;
  data_file.open("quadratic_data.csv", std::ios_base::app);

  const int nq = plant().num_positions();
  const T q1 = state.q()[1](0);
  const T q2 = state.q()[1](1);
  const T dq1 = dq(nq);
  const T dq2 = dq(nq + 1);

  const VectorX<T>& g = EvalGradient(state);
  const MatrixX<T>& H = EvalHessian(state).MakeDense();
  const T g1 = g(nq);
  const T g2 = g(nq + 1);
  const T H11 = H(nq, nq);
  const T H12 = H(nq, nq + 1);
  const T H21 = H(nq + 1, nq);
  const T H22 = H(nq + 1, nq + 1);

  data_file << fmt::format(
      "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n", iter, q1,
      q2, dq1, dq2, Delta, EvalCost(state), g1, g2, H11, H12, H21, H22,
      g.norm(), H.block(2, 2, 2, 2).norm());
  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SaveLinesearchResidual(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state,
    const std::string filename) const {
  double alpha_min = -0.2;
  double alpha_max = 1.2;
  double dalpha = 0.001;

  std::ofstream data_file;
  data_file.open(filename);
  data_file << "alpha, cost, gradient, dq, L_prime \n";  // header

  double alpha = alpha_min;
  while (alpha <= alpha_max) {
    // Record the linesearch parameter alpha
    data_file << alpha << ", ";

    // Record the linesearch residual
    // phi(alpha) = L(q + alpha * dq) - L
    scratch_state->set_q(state.q());
    scratch_state->AddToQ(alpha * dq);
    data_file << EvalCost(*scratch_state) - EvalCost(state) << ", ";

    // Record the norm of the gradient
    const VectorX<T>& g = EvalGradient(*scratch_state);
    data_file << g.norm() << ", ";

    // Record the norm of the search direction
    data_file << dq.norm() << ", ";

    // Record the derivative of the linesearch residual w.r.t alpha
    data_file << g.dot(dq) << "\n";

    alpha += dalpha;
  }
  data_file.close();
}

template <typename T>
std::tuple<double, int> TrajectoryOptimizer<T>::Linesearch(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state) const {
  // The state's cache must be up to date, since we'll use the gradient and cost
  // information stored there.
  if (params_.linesearch_method == LinesearchMethod::kArmijo) {
    return ArmijoLinesearch(state, dq, scratch_state);
  } else if (params_.linesearch_method == LinesearchMethod::kBacktracking) {
    return BacktrackingLinesearch(state, dq, scratch_state);
  } else {
    throw std::runtime_error("Unknown linesearch method");
  }
}

template <typename T>
std::tuple<double, int> TrajectoryOptimizer<T>::BacktrackingLinesearch(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state) const {
  using std::abs;

  // Compute the cost and gradient
  const T L = EvalCost(state);
  const VectorX<T>& g = EvalGradient(state);

  // Linesearch parameters
  const double c = 1e-4;
  const double rho = 0.8;

  double alpha = 1.0;
  T L_prime = g.transpose() * dq;  // gradient of L w.r.t. alpha

  // Make sure this is a descent direction
  DRAKE_DEMAND(L_prime <= 0);

  // Exit early with alpha = 1 when we are close to convergence
  const double convergence_threshold =
      sqrt(std::numeric_limits<double>::epsilon());
  if (abs(L_prime) / abs(L) <= convergence_threshold) {
    return {1.0, 0};
  }

  // Try with alpha = 1
  scratch_state->set_q(state.q());
  scratch_state->AddToQ(alpha * dq);
  T L_old = EvalCost(*scratch_state);

  // L_new stores cost at iteration i:   L(q + alpha_i * dq)
  // L_old stores cost at iteration i-1: L(q + alpha_{i-1} * dq)
  T L_new = L_old;

  // We'll keep reducing alpha until (1) we meet the Armijo convergence
  // criteria and (2) the cost increases, indicating that we're near a local
  // minimum.
  int i = 0;
  bool armijo_met = false;
  while (!(armijo_met && (L_new > L_old))) {
    // Save L_old = L(q + alpha_{i-1} * dq)
    L_old = L_new;

    // Reduce alpha
    alpha *= rho;

    // Compute L_new = L(q + alpha_i * dq)
    scratch_state->set_q(state.q());
    scratch_state->AddToQ(alpha * dq);
    L_new = EvalCost(*scratch_state);

    // Check the Armijo conditions
    if (L_new <= L + c * alpha * L_prime) {
      armijo_met = true;
    }

    ++i;
  }

  return {alpha / rho, i};
}

template <typename T>
std::tuple<double, int> TrajectoryOptimizer<T>::ArmijoLinesearch(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state) const {
  using std::abs;

  // Compute the cost and gradient
  const T L = EvalCost(state);
  const VectorX<T>& g = EvalGradient(state);

  // Linesearch parameters
  const double c = 1e-4;
  const double rho = 0.8;

  double alpha = 1.0 / rho;        // get alpha = 1 on first iteration
  T L_prime = g.transpose() * dq;  // gradient of L w.r.t. alpha
  T L_new;                         // L(q + alpha * dq)

  // Make sure this is a descent direction
  DRAKE_DEMAND(L_prime <= 0);

  // Exit early with alpha = 1 when we are close to convergence
  const double convergence_threshold =
      sqrt(std::numeric_limits<double>::epsilon());
  if (abs(L_prime) / abs(L) <= convergence_threshold) {
    return {1.0, 0};
  }

  int i = 0;  // Iteration counter
  do {
    // Reduce alpha
    // N.B. we start with alpha = 1/rho, so we get alpha = 1 on the first
    // iteration.
    alpha *= rho;

    // Compute L_ls = L(q + alpha * dq)
    scratch_state->set_q(state.q());
    scratch_state->AddToQ(alpha * dq);
    L_new = EvalCost(*scratch_state);

    ++i;
  } while ((L_new > L + c * alpha * L_prime) &&
           (i < params_.max_linesearch_iterations));

  return {alpha, i};
}

template <typename T>
T TrajectoryOptimizer<T>::CalcTrustRatio(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state) const {
  // Compute predicted reduction in cost
  const VectorX<T>& g = EvalGradient(state);
  const PentaDiagonalMatrix<T>& H = EvalHessian(state);
  const T gradient_term = g.dot(dq);
  VectorX<T>& Hdq = state.workspace.q_times_num_steps_size_tmp;
  H.MultiplyBy(dq, &Hdq);
  const T hessian_term = 0.5 * dq.transpose() * Hdq;
  const T predicted_reduction = -gradient_term - hessian_term;

  // Compute actual reduction in cost
  scratch_state->set_q(state.q());
  scratch_state->AddToQ(dq);
  const T L_old = EvalCost(state);           // L(q)
  const T L_new = EvalCost(*scratch_state);  // L(q + dq)
  const T actual_reduction = L_old - L_new;

  const double eps = std::numeric_limits<T>::epsilon();
  if ((predicted_reduction < eps) && (actual_reduction < eps)) {
    // Predicted and actual reduction are essentially zero
    return 1.0;
  }

  return actual_reduction / predicted_reduction;
}

template <typename T>
T TrajectoryOptimizer<T>::SolveDoglegQuadratic(const T& a, const T& b,
                                               const T& c) const {
  using std::sqrt;
  // Check that a is positive
  DRAKE_DEMAND(a > 0);

  T s;
  if (a < std::numeric_limits<double>::epsilon()) {
    // If a is essentially zero, just solve bx + c = 0
    s = -c / b;
  } else {
    // Normalize everything by a
    const T b_tilde = b / a;
    const T c_tilde = c / a;

    const T determinant = b_tilde * b_tilde - 4 * c_tilde;
    DRAKE_DEMAND(determinant > 0);  // We know a real root exists

    // We know that there is only one positive root, so we just take the big
    // root
    s = (-b_tilde + sqrt(determinant)) / 2;
  }

  // We know the solution is between zero and one
  DRAKE_DEMAND(0 < s);
  DRAKE_DEMAND(s < 1);

  return s;
}

template <typename T>
bool TrajectoryOptimizer<T>::CalcDoglegPoint(const TrajectoryOptimizerState<T>&,
                                             const double, VectorX<T>*) const {
  // Only T=double is supported here, since pentadigonal matrix factorization is
  // (sometimes) required to compute the dogleg point.
  throw std::runtime_error(
      "TrajectoryOptimizer::CalcDoglegPoint only supports T=double");
}

template <>
bool TrajectoryOptimizer<double>::CalcDoglegPoint(
    const TrajectoryOptimizerState<double>& state, const double Delta,
    VectorXd* dq) const {
  // N.B. We'll rescale pU and pH by Δ to avoid roundoff error
  const VectorXd& g = EvalGradient(state);
  const PentaDiagonalMatrix<double>& H = EvalHessian(state);
  VectorXd& Hg = state.workspace.q_times_num_steps_size_tmp;
  H.MultiplyBy(g, &Hg);
  const double gHg = g.transpose() * Hg;

  // Compute the unconstrained minimizer of m(δq) = L(q) + g(q)'*δq + 1/2
  // δq'*H(q)*δq along -g
  VectorXd& pU = state.workspace.q_size_tmp1;
  pU = -(g.dot(g) / gHg) * g / Delta;  // normalize by Δ

  // Check if the trust region is smaller than this unconstrained minimizer
  if (1.0 <= pU.norm()) {
    // If so, δq is where the first leg of the dogleg path intersects the trust
    // region.
    *dq = (Delta / pU.norm()) * pU;
    return true;  // the trust region constraint is active
  }

  // Compute the full Gauss-Newton step
  VectorXd& pH = state.workspace.q_size_tmp2;
  pH = -g / Delta;  // normalize by Δ
  PentaDiagonalFactorization Hchol(H);
  DRAKE_DEMAND(Hchol.status() == PentaDiagonalFactorizationStatus::kSuccess);
  Hchol.SolveInPlace(&pH);

  // Check if the trust region is large enough to just take the full Newton step
  if (1.0 >= pH.norm()) {
    *dq = pH * Delta;
    return false;  // the trust region constraint is not active
  }

  // Compute the intersection between the second leg of the dogleg path and the
  // trust region. We'll do this by solving the (scalar) quadratic
  //
  //    ‖ pU + s( pH − pU ) ‖² = y²
  //
  // for s ∈ (0,1),
  //
  // and setting
  //
  //    δq = pU + s( pH − pU ).
  //
  // Note that we normalize by Δ to minimize roundoff error.
  const double a = (pH - pU).dot(pH - pU);
  const double b = 2 * pU.dot(pH - pU);
  const double c = pU.dot(pU) - 1.0;
  const double s = SolveDoglegQuadratic(a, b, c);

  *dq = (pU + s * (pH - pU)) * Delta;

  return true;  // the trust region constraint is active
}

template <typename T>
SolverFlag TrajectoryOptimizer<T>::Solve(const std::vector<VectorX<T>>&,
                                         TrajectoryOptimizerSolution<T>*,
                                         TrajectoryOptimizerStats<T>*) const {
  throw std::runtime_error(
      "TrajectoryOptimizer::Solve only supports T=double.");
}

template <>
SolverFlag TrajectoryOptimizer<double>::Solve(
    const std::vector<VectorXd>& q_guess,
    TrajectoryOptimizerSolution<double>* solution,
    TrajectoryOptimizerStats<double>* stats) const {
  // The guess must be consistent with the initial condition
  DRAKE_DEMAND(q_guess[0] == prob_.q_init);
  DRAKE_DEMAND(static_cast<int>(q_guess.size()) == num_steps() + 1);

  // stats must be empty
  DRAKE_DEMAND(stats->is_empty());

  if (params_.method == SolverMethod::kLinesearch) {
    return SolveWithLinesearch(q_guess, solution, stats);
  } else if (params_.method == SolverMethod::kTrustRegion) {
    return SolveWithTrustRegion(q_guess, solution, stats);
  } else {
    throw std::runtime_error("Unsupported solver strategy!");
  }
}

template <typename T>
SolverFlag TrajectoryOptimizer<T>::SolveWithLinesearch(
    const std::vector<VectorX<T>>&, TrajectoryOptimizerSolution<T>*,
    TrajectoryOptimizerStats<T>*) const {
  throw std::runtime_error(
      "TrajectoryOptimizer::SolveWithLinesearch only supports T=double.");
}

template <>
SolverFlag TrajectoryOptimizer<double>::SolveWithLinesearch(
    const std::vector<VectorXd>& q_guess,
    TrajectoryOptimizerSolution<double>* solution,
    TrajectoryOptimizerStats<double>* stats) const {
  // Allocate a state variable
  TrajectoryOptimizerState<double> state = CreateState();
  state.set_q(q_guess);

  // Allocate a separate state variable for linesearch
  TrajectoryOptimizerState<double> scratch_state = CreateState();

  // Allocate cost and search direction
  double cost;
  VectorXd dq((num_steps() + 1) * plant().num_positions());

  // Set proximal operator data for the first iteration
  // N.B. since state.proximal_operator_data.H_diag is initialized to zero, this
  // first computation of the Hessian, which is for scaling purposes only, will
  // not include the proximal operator term.
  if (params_.proximal_operator) {
    state.set_proximal_operator_data(q_guess, EvalHessian(state));
    scratch_state.set_proximal_operator_data(q_guess, EvalHessian(state));
  }

  if (params_.verbose) {
    // Define printout data
    std::cout << "-------------------------------------------------------------"
                 "---------"
              << std::endl;
    std::cout << "|  iter  |   cost   |  alpha  |  LS_iters  |  time (s)  |  "
                 "|g|/cost  |"
              << std::endl;
    std::cout << "-------------------------------------------------------------"
                 "---------"
              << std::endl;
  }

  // Allocate timing variables
  auto start_time = std::chrono::high_resolution_clock::now();
  auto iter_start_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> iter_time;
  std::chrono::duration<double> solve_time;

  // Gauss-Newton iterations
  int k = 0;                       // iteration counter
  bool linesearch_failed = false;  // linesearch success flag
  do {
    iter_start_time = std::chrono::high_resolution_clock::now();

    // Compute the total cost
    cost = EvalCost(state);

    // Compute gradient and Hessian
    const VectorXd& g = EvalGradient(state);
    const PentaDiagonalMatrix<double>& H = EvalHessian(state);

    // Solve for search direction H*dq = -g
    dq = -g;
    PentaDiagonalFactorization Hchol(H);
    if (Hchol.status() != PentaDiagonalFactorizationStatus::kSuccess) {
      return SolverFlag::kFactorizationFailed;
    }
    Hchol.SolveInPlace(&dq);

    // Solve the linsearch
    // N.B. we use a separate state variable since we will need to compute
    // L(q+alpha*dq) (at the very least), and we don't want to change state.q
    auto [alpha, ls_iters] = Linesearch(state, dq, &scratch_state);

    // Record linesearch data, if requested
    if (params_.linesearch_plot_every_iteration) {
      SaveLinesearchResidual(state, dq, &scratch_state,
                             fmt::format("linesearch_data_{}.csv", k));
    }

    if (ls_iters >= params_.max_linesearch_iterations) {
      linesearch_failed = true;

      if (params_.verbose) {
        std::cout << "LINESEARCH FAILED" << std::endl;
        std::cout << "Reached maximum linesearch iterations ("
                  << params_.max_linesearch_iterations << ")." << std::endl;
      }

      // Save the linesearch residual to a csv file so we can plot in python
      SaveLinesearchResidual(state, dq, &scratch_state);
    }

    // Compute the trust ratio (actual cost reduction / model cost reduction)
    double trust_ratio = CalcTrustRatio(state, alpha * dq, &scratch_state);

    // Update the decision variables
    state.AddToQ(alpha * dq);

    // Update the stored decision variables for the proximal operator cost
    if (params_.proximal_operator) {
      state.set_proximal_operator_data(state.q(), H);
      scratch_state.set_proximal_operator_data(state.q(), H);
    }

    iter_time = std::chrono::high_resolution_clock::now() - iter_start_time;

    // Nice little printout of our problem data
    if (params_.verbose) {
      printf("| %6d ", k);
      printf("| %8.3f ", cost);
      printf("| %7.4f ", alpha);
      printf("| %6d     ", ls_iters);
      printf("| %8.8f ", iter_time.count());
      printf("| %10.3e |\n", g.norm() / cost);
    }

    // Print additional debuging information
    if (params_.print_debug_data) {
      double condition_number = 1 / H.MakeDense().ldlt().rcond();
      double L_prime = g.transpose() * dq;
      std::cout << "Condition #: " << condition_number << std::endl;
      std::cout << "|| dq ||   : " << dq.norm() << std::endl;
      std::cout << "||  g ||   : " << g.norm() << std::endl;
      std::cout << "L'         : " << L_prime << std::endl;
      std::cout << "L          : " << cost << std::endl;
      std::cout << "L' / L     : " << L_prime / cost << std::endl;
      std::cout << "||diag(H)||: " << H.MakeDense().diagonal().norm()
                << std::endl;
      if (k > 0) {
        std::cout << "L[k] - L[k-1]: " << cost - stats->iteration_costs[k - 1]
                  << std::endl;
      }
    }

    // Record iteration data
    stats->push_data(iter_time.count(),  // iteration time
                     cost,               // cost
                     ls_iters,           // sub-problem iterations
                     alpha,              // linesearch parameter
                     NAN,                // trust region size
                     dq.norm(),          // step size
                     trust_ratio,        // trust ratio
                     g.norm());          // gradient size

    ++k;
  } while (k < params_.max_iterations && !linesearch_failed);

  // End the problem data printout
  if (params_.verbose) {
    std::cout << "-------------------------------------------------------------"
                 "---------"
              << std::endl;
  }

  // Record the total solve time
  solve_time = std::chrono::high_resolution_clock::now() - start_time;
  stats->solve_time = solve_time.count();

  // Record the solution
  solution->q = state.q();
  solution->v = EvalV(state);
  solution->tau = EvalTau(state);

  if (linesearch_failed) {
    return SolverFlag::kLinesearchMaxIters;
  } else {
    return SolverFlag::kSuccess;
  }
}

template <typename T>
SolverFlag TrajectoryOptimizer<T>::SolveWithTrustRegion(
    const std::vector<VectorX<T>>&, TrajectoryOptimizerSolution<T>*,
    TrajectoryOptimizerStats<T>*) const {
  throw std::runtime_error(
      "TrajectoryOptimizer::SolveWithTrustRegion only supports T=double.");
}

template <>
SolverFlag TrajectoryOptimizer<double>::SolveWithTrustRegion(
    const std::vector<VectorXd>& q_guess,
    TrajectoryOptimizerSolution<double>* solution,
    TrajectoryOptimizerStats<double>* stats) const {
  using std::min;
  // Allocate a state variable to store q and everything that is computed from q
  TrajectoryOptimizerState<double> state = CreateState();
  state.set_q(q_guess);

  // Allocate a separate state variable for computations like L(q + dq)
  TrajectoryOptimizerState<double> scratch_state = CreateState();

  // Allocate the update vector q_{k+1} = q_k + dq
  VectorXd dq(plant().num_positions() * (num_steps() + 1));

  // Set up a file to record iteration data for a contour plot
  if (params_.save_contour_data) {
    SetupQuadraticDataFile();
  }
  if (params_.save_lineplot_data) {
    SetupIterationDataFile();
  }

  // Allocate timing variables
  auto start_time = std::chrono::high_resolution_clock::now();
  auto iter_start_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> iter_time;
  std::chrono::duration<double> solve_time;

  // Trust region parameters
  const double Delta_max = 1.0;  // Maximum trust region size
  const double Delta0 = 1e0;     // Initial trust region size
  const double eta = 0.0;        // Trust ratio threshold - we accept steps if
                                 // the trust ratio is above this threshold

  // Variables that we'll update throughout the main loop
  int k = 0;                  // iteration counter
  double Delta = Delta0;      // trust region size
  double rho;                 // trust region ratio
  bool tr_constraint_active;  // flag for whether the trust region constraint is
                              // active

  // Define printout data
  const std::string separator_bar =
      "-----------------------------------------------------------------------"
      "-";
  const std::string printout_labels =
      "|  iter  |   cost   |    Δ    |    ρ    |  time (s)  |  |g|/cost  |";

  while (k < params_.max_iterations) {
    // Obtain the candiate update dq
    tr_constraint_active = CalcDoglegPoint(state, Delta, &dq);

    // Compute the trust region ratio
    rho = CalcTrustRatio(state, dq, &scratch_state);

    // Save data related to our quadratic approximation (for the first two
    // variables)
    if (params_.save_contour_data) {
      SaveQuadraticDataFirstTwoVariables(k, Delta, dq, state);
    }
    if (params_.save_lineplot_data) {
      SaveIterationData(k, Delta, rho, dq(1), state);
    }

    // If the ratio is large enough, accept the change
    if (rho > eta) {
      // Update the coefficients for the proximal operator cost
      if (params_.proximal_operator) {
        state.set_proximal_operator_data(state.q(), EvalHessian(state));
        scratch_state.set_proximal_operator_data(state.q(), EvalHessian(state));
      }

      state.AddToQ(dq);  // q += dq
    }
    // Else (rho <= eta), the trust region ratio is too small to accept dq, so
    // we'll need to so keep reducing the trust region. Note that the trust
    // region will be reduced in this case, since eta < 0.25.

    // N.B. if this is the case (q_{k+1} = q_k), we haven't touched state, so we
    // should be reusing the cached gradient and Hessian in the next iteration.
    // TODO(vincekurtz): should we be caching the factorization of the Hessian,
    // as well as the Hessian itself?

    // Compute iteration timing
    // N.B. this is in kind of a weird place because we want to record
    // statistics before updating the trust-region size. That ensures that
    // ‖ δq ‖ ≤ Δ in our logs.
    iter_time = std::chrono::high_resolution_clock::now() - iter_start_time;
    iter_start_time = std::chrono::high_resolution_clock::now();

    // Printout statistics from this iteration
    if (params_.verbose) {
      if ((k % 50) == 0) {
        // Refresh the labels for easy reading
        std::cout << separator_bar << std::endl;
        std::cout << printout_labels << std::endl;
        std::cout << separator_bar << std::endl;
      }
      printf("| %6d ", k);
      printf("| %8.3f ", EvalCost(state));
      printf("| %7.1e ", Delta);
      printf("| %7.4f ", rho);
      printf("| %8.8f ", iter_time.count());
      printf("| %10.3e |\n", EvalGradient(state).norm() / EvalCost(state));
    }

    // Record statistics from this iteration
    stats->push_data(iter_time.count(),            // iteration time
                     EvalCost(state),              // cost
                     0,                            // linesearch iterations
                     NAN,                          // linesearch parameter
                     Delta,                        // trust region size
                     dq.norm(),                    // step size
                     rho,                          // trust region ratio
                     EvalGradient(state).norm());  // gradient size

    // Update the size of the trust-region, if necessary
    if (rho < 0.25) {
      // If the ratio is small, our quadratic approximation is bad, so reduce
      // the trust region
      Delta *= 0.25;
    } else if ((rho > 0.75) && tr_constraint_active) {
      // If the ratio is large and we're at the boundary of the trust
      // region, increase the size of the trust region.
      Delta = min(2 * Delta, Delta_max);
    }

    ++k;
  }

  // Finish our printout
  if (params_.verbose) {
    std::cout << separator_bar << std::endl;
  }

  solve_time = std::chrono::high_resolution_clock::now() - start_time;
  stats->solve_time = solve_time.count();

  // Record the solution
  solution->q = state.q();
  solution->v = EvalV(state);
  solution->tau = EvalTau(state);

  // Record L(q) for various values of q so we can make plots
  if (params_.save_contour_data) {
    SaveContourPlotDataFirstTwoVariables(&scratch_state);
  }
  if (params_.save_lineplot_data) {
    SaveLinePlotDataFirstVariable(&scratch_state);
  }

  return SolverFlag::kSuccess;
}

}  // namespace traj_opt
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::traj_opt::TrajectoryOptimizer)
