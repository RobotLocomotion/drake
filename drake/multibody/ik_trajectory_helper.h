#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/multibody/ik_options.h"

namespace drake {
namespace systems {
namespace plants {

/// This class is a helper for backend implementations of inverse
/// kinematics trajectory planning.  It holds some general data about
/// the problem being computed and is able to calculate some of the
/// interesting parts of the optimization.  It is intended to only
/// hold/calculate data which is independent of the solver being used.
class IKTrajectoryHelper  {
 public:
  /// @param nq -- number of positions of the body
  /// @param nT -- number of timesteps being planned
  /// @param t -- array of timestep values (of length @p nT)
  /// @param num_qfree -- number of timesteps for which position
  ///                     variables are free to change during
  ///                     optimization.
  /// @param num_qdotfree -- number of qdot states which are free to
  ///                        change during optimization (in practice,
  ///                        this will be 1 (final only) when the
  ///                        initial state is fixed, and 2 (initial
  ///                        and final) otherwise.
  /// @param ikoptions -- options passed to inverseKinTraj
  /// @param dt -- array of deltas between timestamps (length nT - 1)
  /// @param dt_ratio -- ratio of consecutive values in dt (length nT - 2)
  IKTrajectoryHelper(int nq, int nT, const double* t,
                     int num_qfree, int num_qdotfree,
                     const IKoptions& ikoptions,
                     const double* dt, const double* dt_ratio);

  /// Calculate the cost given the current @p q, @p qdot0, and @p qdotf.
  ///
  /// @return cost
  /// @param q -- Current position estimates
  /// @param q_nom -- Estimates for nominal postures
  /// @param qstart_idx -- offset into q to start considering in cost
  ///                      calculations (usually 0 if the initial
  ///                      state is free, and 1 otherwise)
  /// @param qdot0 -- current estimate of qdot at timestep 0
  /// @param qdotf -- current estimate of qdot at final timestep
  /// @param fix_initial_state -- true if the initial state is fixed
  /// @param[out] dJ_vec Matrix to be filled in with the current
  ///                    gradients.
  double CalculateCost(const Eigen::MatrixXd& q, const Eigen::MatrixXd& q_nom,
                       int qstart_idx, const Eigen::VectorXd& qdot0,
                       const Eigen::VectorXd& qdotf, bool fix_initial_state,
                       Eigen::MatrixXd* dJ_vec) const;

  int nq() const { return nq_; }
  int nT() const { return nT_; }
  const double* t() const { return t_; }
  int num_qfree() const { return num_qfree_; }
  int num_qdotfree() const { return num_qdotfree_; }

  const Eigen::MatrixXd& Q() const { return Q_; }
  const Eigen::MatrixXd& Qa() const { return Qa_; }
  const Eigen::MatrixXd& Qv() const { return Qv_; }
  const Eigen::MatrixXd& velocity_mat() const { return velocity_mat_; }
  const Eigen::MatrixXd& velocity_mat_qd0() const { return velocity_mat_qd0_; }
  const Eigen::MatrixXd& velocity_mat_qdf() const { return velocity_mat_qdf_; }
  const Eigen::MatrixXd& accel_mat() const { return accel_mat_; }
  const Eigen::MatrixXd& accel_mat_qd0() const { return accel_mat_qd0_; }
  const Eigen::MatrixXd& accel_mat_qdf() const { return accel_mat_qdf_; }

  const std::vector<Eigen::VectorXd>& t_inbetween() const {
    return t_inbetween_;
  }
  const std::vector<double>& t_samples() const { return t_samples_; }
  const std::vector<Eigen::MatrixXd>& dq_inbetween_dqknot() const {
    return dq_inbetween_dqknot_;
  }
  const std::vector<Eigen::MatrixXd>& dq_inbetween_dqd0() const {
    return dq_inbetween_dqd0_;
  }
  const std::vector<Eigen::MatrixXd>& dq_inbetween_dqdf() const {
    return dq_inbetween_dqdf_;
  }

 private:
  const int nq_;
  const int nT_;
  const double* t_;
  const int num_qfree_;
  const int num_qdotfree_;

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd Qa_;
  Eigen::MatrixXd Qv_;

  Eigen::MatrixXd velocity_mat_;
  Eigen::MatrixXd velocity_mat_qd0_;
  Eigen::MatrixXd velocity_mat_qdf_;
  Eigen::MatrixXd accel_mat_;
  Eigen::MatrixXd accel_mat_qd0_;
  Eigen::MatrixXd accel_mat_qdf_;

  std::vector<Eigen::VectorXd> t_inbetween_;
  std::vector<double> t_samples_;
  int num_inbetween_t_samples_;

  std::vector<Eigen::MatrixXd> dq_inbetween_dqknot_;
  std::vector<Eigen::MatrixXd> dq_inbetween_dqd0_;
  std::vector<Eigen::MatrixXd> dq_inbetween_dqdf_;
};

}  // namespace plants
}  // namespace systems
}  // namespace drake
