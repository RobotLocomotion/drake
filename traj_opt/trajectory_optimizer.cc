#include "drake/traj_opt/trajectory_optimizer.h"

#include <algorithm>
#include <iostream>
#include <limits>

namespace drake {
namespace traj_opt {

using multibody::Joint;
using multibody::JointIndex;
using multibody::MultibodyPlant;
using systems::System;

TrajectoryOptimizer::TrajectoryOptimizer(const MultibodyPlant<double>* plant,
                                         const ProblemDefinition& prob)
    : plant_(plant), prob_(prob) {
  context_ = plant_->CreateDefaultContext();

  // Define joint damping coefficients.
  joint_damping_ = VectorXd::Zero(plant_->num_velocities());

  for (JointIndex j(0); j < plant_->num_joints(); ++j) {
    const Joint<double>& joint = plant_->get_joint(j);
    const int velocity_start = joint.velocity_start();
    const int nv = joint.num_velocities();
    joint_damping_.segment(velocity_start, nv) = joint.damping_vector();
  }
}

double TrajectoryOptimizer::CalcCost(
    const std::vector<VectorXd>& q, const std::vector<VectorXd>& v,
    const std::vector<VectorXd>& tau,
    TrajectoryOptimizerWorkspace* workspace) const {
  double cost = 0;

  VectorXd& q_err = workspace->q_size_tmp;
  VectorXd& v_err = workspace->v_size_tmp1;

  // Running cost
  for (int t = 0; t < num_steps(); ++t) {
    q_err = q[t] - prob_.q_nom;
    v_err = v[t] - prob_.v_nom;
    cost += q_err.transpose() * prob_.Qq * q_err;
    cost += v_err.transpose() * prob_.Qv * v_err;
    cost += tau[t].transpose() * prob_.R * tau[t];
  }

  // Scale running cost by dt (so the optimization problem we're solving doesn't
  // change so dramatically when we change the time step).
  cost *= time_step();

  // Terminal cost
  q_err = q[num_steps()] - prob_.q_nom;
  v_err = v[num_steps()] - prob_.v_nom;
  cost += q_err.transpose() * prob_.Qf_q * q_err;
  cost += v_err.transpose() * prob_.Qf_v * v_err;

  return cost;
}

double TrajectoryOptimizer::CalcCost(
    const std::vector<VectorXd>& q,
    TrajectoryOptimizerWorkspace* workspace) const {
  // These are expensive heap allocations: prefer versions of CalcCost that
  // use precomputed v and tau whenever possible.
  std::vector<VectorXd> v(num_steps() + 1);
  std::vector<VectorXd> tau(num_steps());

  CalcV(q, &v);
  CalcTau(q, v, workspace, &tau);

  return CalcCost(q, v, tau, workspace);
}

void TrajectoryOptimizer::CalcV(const std::vector<VectorXd>& q,
                                std::vector<VectorXd>* v) const {
  // x = [x0, x1, ..., xT]
  DRAKE_DEMAND(static_cast<int>(q.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(v->size()) == num_steps() + 1);

  v->at(0) = prob_.v_init;
  for (int i = 1; i <= num_steps(); ++i) {
    v->at(i) = (q[i] - q[i - 1]) / time_step();
  }
}

void TrajectoryOptimizer::CalcTau(const std::vector<VectorXd>& q,
                                  const std::vector<VectorXd>& v,
                                  TrajectoryOptimizerWorkspace* workspace,
                                  std::vector<VectorXd>* tau) const {
  // Generalized forces aren't defined for the last timestep
  // TODO(vincekurtz): additional checks that q_t, v_t, tau_t are the right size
  // for the plant?
  DRAKE_DEMAND(static_cast<int>(q.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(v.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(tau->size()) == num_steps());

  for (int t = 0; t < num_steps(); ++t) {
    // Acceleration at time t
    VectorXd& a = workspace->a_size_tmp1;
    a = (v[t + 1] - v[t]) / time_step();

    // All dynamics terms are treated implicitly, i.e.,
    // tau[t] = M(q[t+1]) * a[t] - k(q[t+1],v[t+1]) - f_ext[t+1]
    CalcInverseDynamics(q[t + 1], v[t + 1], a, workspace, &tau->at(t));
  }
}

void TrajectoryOptimizer::CalcInverseDynamics(
    const VectorXd& q, const VectorXd& v, const VectorXd& a,
    TrajectoryOptimizerWorkspace* workspace, VectorXd* tau) const {
  plant().SetPositions(context_.get(), q);
  plant().SetVelocities(context_.get(), v);
  plant().CalcForceElementsContribution(*context_, &workspace->f_ext);

  // Inverse dynamics computes tau = M*a - k(q,v) - f_ext
  *tau = plant().CalcInverseDynamics(*context_, a, workspace->f_ext);

  // TODO(vincekurtz) add in contact/constriant contribution
}

void TrajectoryOptimizer::CalcInverseDynamicsPartials(
    const std::vector<VectorXd>& q, const std::vector<VectorXd>& v,
    TrajectoryOptimizerWorkspace* workspace, GradientData* grad_data) const {
  // TODO(vincekurtz): use a solver flag to choose between finite differences
  // and an analytical approximation
  CalcInverseDynamicsPartialsFiniteDiff(q, v, workspace, grad_data);
}

void TrajectoryOptimizer::CalcInverseDynamicsPartialsFiniteDiff(
    const std::vector<VectorXd>& q, const std::vector<VectorXd>& v,
    TrajectoryOptimizerWorkspace* workspace, GradientData* grad_data) const {
  // Check that grad_data has been allocated correctly.
  DRAKE_DEMAND(grad_data->size() == num_steps());

  // Get references to the partials that we'll be setting
  std::vector<MatrixXd>& dtau_dqm = grad_data->dtau_dqm;
  std::vector<MatrixXd>& dtau_dqt = grad_data->dtau_dqt;
  std::vector<MatrixXd>& dtau_dqp = grad_data->dtau_dqp;

  // Compute tau(q) [all timesteps] using the orignal value of q
  // TODO(vincekurtz): consider passing this as an argument along with q and v,
  // perhaps combined into a TrajectoryData struct
  std::vector<VectorXd> tau(num_steps());
  CalcTau(q, v, workspace, &tau);

  // Compute a(q) [all timesteps] store (unperturbed) accelerations
  // TODO(vincekurt): put this all in the same object as q, v, and
  // tau. Possibly with a State[q]/Cache[v,a,tau,dtau/dq] structure.
  std::vector<VectorXd> a(num_steps());
  for (int t = 0; t < num_steps(); ++t) {
    a[t] = (v[t + 1] - v[t]) / time_step();
  }

  // Get references to perturbed versions of q, v, tau, and a, at (t-1, t, t).
  // These are all of the quantities that change when we perturb q_t.
  VectorXd& q_eps_t = workspace->q_size_tmp;
  VectorXd& v_eps_t = workspace->v_size_tmp1;
  VectorXd& v_eps_tp = workspace->v_size_tmp2;
  VectorXd& a_eps_tm = workspace->a_size_tmp1;
  VectorXd& a_eps_t = workspace->a_size_tmp2;
  VectorXd& a_eps_tp = workspace->a_size_tmp3;
  VectorXd& tau_eps_tm = workspace->tau_size_tmp1;
  VectorXd& tau_eps_t = workspace->tau_size_tmp2;
  VectorXd& tau_eps_tp = workspace->tau_size_tmp3;

  // Store small perturbations
  const double eps = sqrt(std::numeric_limits<double>::epsilon());
  double dq_i;
  double dv_i;
  double da_i;
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
      dq_i = eps * std::max(1.0, std::abs(q_eps_t(i)));
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
        CalcInverseDynamics(q_eps_t, v_eps_t, a_eps_tm, workspace, &tau_eps_tm);
        dtau_dqp[t - 1].col(i) = (tau_eps_tm - tau[t - 1]) / dq_i;
      }
      if (t < num_steps()) {
        // tau[t] = ID(q[t+1], v[t+1], a[t])
        CalcInverseDynamics(q[t + 1], v_eps_tp, a_eps_t, workspace, &tau_eps_t);
        dtau_dqt[t].col(i) = (tau_eps_t - tau[t]) / dq_i;
      }
      if (t < num_steps() - 1) {
        // tau[t+1] = ID(q[t+2], v[t+2], a[t+1])
        CalcInverseDynamics(q[t + 2], v[t + 2], a_eps_tp, workspace,
                            &tau_eps_tp);
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

void TrajectoryOptimizer::CalcGradientFiniteDiff(
    const std::vector<VectorXd>& q, TrajectoryOptimizerWorkspace* workspace,
    EigenPtr<VectorXd> g) const {
  // Allocate perturbed versions of q
  // TODO(vincekurtz): consider allocating in workspace
  std::vector<VectorXd> q_plus(q);
  std::vector<VectorXd> q_minus(q);

  // Set first block of g (derivatives w.r.t. q_0) to zero, since q0 = q_init
  // are constant.
  g->topRows(plant().num_positions()).setZero();

  // Iterate through rows of g using finite differences
  const double eps = cbrt(std::numeric_limits<double>::epsilon());
  double dqt_i;
  int j = plant().num_positions();
  for (int t = 1; t <= num_steps(); ++t) {
    for (int i = 0; i < plant().num_positions(); ++i) {
      // Set finite difference step size
      dqt_i = eps * std::max(1.0, std::abs(q[t](i)));
      q_plus[t](i) += dqt_i;
      q_minus[t](i) -= dqt_i;

      // Set g_j = using central differences
      double L_plus = CalcCost(q_plus, workspace);
      double L_minus = CalcCost(q_minus, workspace);
      (*g)(j) = (L_plus - L_minus) / (2 * dqt_i);

      // reset our perturbed Q and move to the next row of g.
      q_plus[t](i) = q[t](i);
      q_minus[t](i) = q[t](i);
      ++j;
    }
  }
}

void TrajectoryOptimizer::CalcGradient(const std::vector<VectorXd>& q,
                                       const GradientData& grad_data,
                                       TrajectoryOptimizerWorkspace* workspace,
                                       EigenPtr<VectorXd> g) const {
  const double dt = time_step();
  const int nq = plant().num_positions();

  // Compute v and tau
  // TODO(vincekurtz): wrap all this data into a TrajectoryOptimizerState and
  // compute ahead of time
  std::vector<VectorXd> v(num_steps() + 1);
  std::vector<VectorXd> tau(num_steps());
  CalcV(q, &v);
  CalcTau(q, v, workspace, &tau);

  // TODO(vincekurtz) this should also go in the TrajectoryOptimizerState (cache
  // part)
  double dvt_dqt = 1 / time_step();  // assuming no quaternion DoFs
  double dvt_dqm = -1 / time_step();

  // Set first block of g (derivatives w.r.t. q_0) to zero, since q0 = q_init
  // are constant.
  g->topRows(plant().num_positions()).setZero();

  // Scratch variables for storing intermediate cost terms
  VectorXd& qt_term = workspace->q_size_tmp;
  VectorXd& vt_term = workspace->v_size_tmp1;
  VectorXd& vp_term = workspace->v_size_tmp2;
  VectorXd& taum_term = workspace->tau_size_tmp1;
  VectorXd& taut_term = workspace->tau_size_tmp2;
  VectorXd& taup_term = workspace->tau_size_tmp3;

  for (int t = 1; t < num_steps(); ++t) {
    // Contribution from position cost
    qt_term = (q[t] - prob_.q_nom).transpose() * 2 * prob_.Qq * dt;

    // Contribution from velocity cost
    vt_term = (v[t] - prob_.v_nom).transpose() * 2 * prob_.Qv * dt * dvt_dqt;
    if (t == num_steps() - 1) {
      // The terminal cost needs to be handled differently
      vp_term = (v[t + 1] - prob_.v_nom).transpose() * 2 * prob_.Qf_v * dvt_dqm;
    } else {
      vp_term =
          (v[t + 1] - prob_.v_nom).transpose() * 2 * prob_.Qv * dt * dvt_dqm;
    }

    // Contribution from control cost
    taum_term =
        tau[t - 1].transpose() * 2 * prob_.R * dt * grad_data.dtau_dqp[t - 1];
    taut_term = tau[t].transpose() * 2 * prob_.R * dt * grad_data.dtau_dqt[t];
    if (t == num_steps() - 1) {
      // There is no constrol input at the final timestep
      taup_term.setZero(nq);
    } else {
      taup_term =
          tau[t + 1].transpose() * 2 * prob_.R * dt * grad_data.dtau_dqm[t + 1];
    }

    // Put it all together to get the gradient w.r.t q[t]
    g->segment(t * nq, nq) =
        qt_term + vt_term + vp_term + taum_term + taut_term + taup_term;
  }

  // Last step is different, because there is terminal cost and v[t+1] doesn't
  // exist
  taum_term = tau[num_steps() - 1].transpose() * 2 * prob_.R * dt *
              grad_data.dtau_dqp[num_steps() - 1];
  qt_term = (q[num_steps()] - prob_.q_nom).transpose() * 2 * prob_.Qf_q;
  vt_term =
      (v[num_steps()] - prob_.v_nom).transpose() * 2 * prob_.Qf_v * dvt_dqt;
  g->tail(nq) = qt_term + vt_term + taum_term;
}

}  // namespace traj_opt
}  // namespace drake
