#include "drake/traj_opt/trajectory_optimizer.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>

#include "drake/traj_opt/penta_diagonal_solver.h"

namespace drake {
namespace traj_opt {

using internal::PentaDiagonalFactorization;
using internal::PentaDiagonalFactorizationStatus;
using multibody::Joint;
using multibody::JointIndex;
using multibody::MultibodyPlant;
using systems::System;

template <typename T>
TrajectoryOptimizer<T>::TrajectoryOptimizer(const MultibodyPlant<T>* plant,
                                            const ProblemDefinition& prob,
                                            const SolverParameters& params)
    : plant_(plant), prob_(prob), params_(params) {
  // Create a context for dynamics computations
  context_ = plant_->CreateDefaultContext();

  // Define joint damping coefficients.
  joint_damping_ = VectorX<T>::Zero(plant_->num_velocities());

  for (JointIndex j(0); j < plant_->num_joints(); ++j) {
    const Joint<T>& joint = plant_->get_joint(j);
    const int velocity_start = joint.velocity_start();
    const int nv = joint.num_velocities();
    joint_damping_.segment(velocity_start, nv) = joint.damping_vector();
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
  return CalcCost(state.q(), v, tau, &state.workspace);
}

template <typename T>
T TrajectoryOptimizer<T>::CalcCost(
    const std::vector<VectorX<T>>& q, const std::vector<VectorX<T>>& v,
    const std::vector<VectorX<T>>& tau,
    TrajectoryOptimizerWorkspace<T>* workspace) const {
  T cost = 0;
  VectorX<T>& q_err = workspace->q_size_tmp;
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
  plant().SetPositions(context_.get(), q);
  plant().SetVelocities(context_.get(), v);
  plant().CalcForceElementsContribution(*context_, &workspace->f_ext);
  // Inverse dynamics computes tau = M*a - k(q,v) - f_ext
  *tau = plant().CalcInverseDynamics(*context_, a, workspace->f_ext);

  // TODO(vincekurtz) add in contact/constriant contribution
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartials(
    const std::vector<VectorX<T>>& q, const std::vector<VectorX<T>>& v,
    const std::vector<VectorX<T>>& a, const std::vector<VectorX<T>>& tau,
    TrajectoryOptimizerWorkspace<T>* workspace,
    InverseDynamicsPartials<T>* id_partials) const {
  // TODO(vincekurtz): use a solver flag to choose between finite differences
  // and an analytical approximation
  CalcInverseDynamicsPartialsFiniteDiff(q, v, a, tau, workspace, id_partials);
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartialsFiniteDiff(
    const std::vector<VectorX<T>>& q, const std::vector<VectorX<T>>& v,
    const std::vector<VectorX<T>>& a, const std::vector<VectorX<T>>& tau,
    TrajectoryOptimizerWorkspace<T>* workspace,
    InverseDynamicsPartials<T>* id_partials) const {
  using std::abs;
  using std::max;
  // Check that id_partials has been allocated correctly.
  DRAKE_DEMAND(id_partials->size() == num_steps());

  // Get references to the partials that we'll be setting
  std::vector<MatrixX<T>>& dtau_dqm = id_partials->dtau_dqm;
  std::vector<MatrixX<T>>& dtau_dqt = id_partials->dtau_dqt;
  std::vector<MatrixX<T>>& dtau_dqp = id_partials->dtau_dqp;

  // Get references to perturbed versions of q, v, tau, and a, at (t-1, t, t).
  // These are all of the quantities that change when we perturb q_t.
  VectorX<T>& q_eps_t = workspace->q_size_tmp;
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
  TrajectoryOptimizerState<T> state_eps(state);

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
  VectorX<T>& qt_term = workspace->q_size_tmp;
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
const std::vector<VectorX<T>>& TrajectoryOptimizer<T>::EvalV(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().trajectory_data.up_to_date)
    CalcCacheTrajectoryData(state);
  return state.cache().trajectory_data.v;
}

template <typename T>
const std::vector<VectorX<T>>& TrajectoryOptimizer<T>::EvalA(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().trajectory_data.up_to_date)
    CalcCacheTrajectoryData(state);
  return state.cache().trajectory_data.a;
}

template <typename T>
const std::vector<VectorX<T>>& TrajectoryOptimizer<T>::EvalTau(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().trajectory_data.up_to_date)
    CalcCacheTrajectoryData(state);
  return state.cache().trajectory_data.tau;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcCacheDerivativesData(
    const TrajectoryOptimizerState<T>& state) const {
  TrajectoryOptimizerCache<T>& cache = state.mutable_cache();
  TrajectoryOptimizerWorkspace<T>& workspace = state.workspace;

  // Get the trajectory data
  const std::vector<VectorX<T>>& q = state.q();
  const std::vector<VectorX<T>>& v = EvalV(state);
  const std::vector<VectorX<T>>& a = EvalA(state);
  const std::vector<VectorX<T>>& tau = EvalTau(state);

  // Some aliases
  InverseDynamicsPartials<T>& id_partials = cache.derivatives_data.id_partials;
  VelocityPartials<T>& v_partials = cache.derivatives_data.v_partials;

  // Compute partial derivatives of inverse dynamics d(tau)/d(q)
  CalcInverseDynamicsPartials(q, v, a, tau, &workspace, &id_partials);

  // Compute partial derivatives of velocities d(v)/d(q)
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
void TrajectoryOptimizer<T>::SaveLinesearchResidual(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state) const {
  double alpha_min = -0.2;
  double alpha_max = 1.2;
  double dalpha = 0.01;

  std::ofstream data_file;
  data_file.open("linesearch_data.csv");
  data_file << "alpha, residual\n";  // header

  double alpha = alpha_min;
  while (alpha <= alpha_max) {
    // Record the linesearch parameter alpha
    data_file << alpha << ", ";

    // Record the linesearch residual
    // phi(alpha) = L(q + alpha * dq) - L
    scratch_state->set_q(state.q());
    scratch_state->AddToQ(alpha * dq);
    data_file << EvalCost(*scratch_state) - EvalCost(state) << "\n";

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
  // TODO(vincekurtz): set these in SolverOptions
  const double c = 1e-4;
  const double rho = 0.9;

  double alpha = 1.0;
  T L_prime = g.transpose() * dq;  // gradient of L w.r.t. alpha

  // Make sure this is a descent direction
  DRAKE_DEMAND(L_prime <= 0);

  // Exit early with alpha = 1 when we are close to convergence
  // N.B. |g|/L converges to only around 1e-8 (due to finite differences), but
  // L'/L appears to converge to something considerably smaller.
  const double convergence_threshold =
      100 * std::numeric_limits<double>::epsilon();
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
  while (!armijo_met || (L_new < L_old)) {
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
  // TODO(vincekurtz): set these in SolverOptions
  const double c = 1e-4;
  const double rho = 0.8;

  double alpha = 1.0 / rho;        // get alpha = 1 on first iteration
  T L_prime = g.transpose() * dq;  // gradient of L w.r.t. alpha
  T L_new;                         // L(q + alpha * dq)

  // Make sure this is a descent direction
  DRAKE_DEMAND(L_prime <= 0);

  // Exit early with alpha = 1 when we are close to convergence
  const double convergence_threshold =
      100 * std::numeric_limits<double>::epsilon();
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

  // Allocate a state variable
  TrajectoryOptimizerState<double> state = CreateState();
  state.set_q(q_guess);

  // Allocate a separate state variable for linesearch
  TrajectoryOptimizerState<double> scratch_state(state);

  // Allocate cost and search direction
  double cost;
  VectorXd dq((num_steps() + 1) * plant().num_positions());

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
  int k = 0;  // iteration counter
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

    if (ls_iters >= params_.max_linesearch_iterations) {
      // Early termination if linesearch is taking too long
      if (params_.verbose) {
        std::cout << "LINESEARCH FAILED" << std::endl;
        std::cout << "Reached maximum linesearch iterations ("
                  << params_.max_linesearch_iterations << ")." << std::endl;
      }

      // Save the linesearch residual to a csv file so we can plot in python
      SaveLinesearchResidual(state, dq, &scratch_state);

      // We'll still record iteration data for playback later
      iter_time = std::chrono::high_resolution_clock::now() - iter_start_time;
      solve_time = std::chrono::high_resolution_clock::now() - start_time;
      stats->solve_time = solve_time.count();
      stats->push_data(iter_time.count(), cost, ls_iters, alpha, g.norm());

      return SolverFlag::kLinesearchMaxIters;
    }

    // Update the decision variables
    state.AddToQ(alpha * dq);

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

    // Record iteration data
    stats->push_data(iter_time.count(), cost, ls_iters, alpha, g.norm());

    ++k;
  } while (k < params_.max_iterations);

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

  return SolverFlag::kSuccess;
}

}  // namespace traj_opt
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::traj_opt::TrajectoryOptimizer)
