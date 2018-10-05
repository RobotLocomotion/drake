#include "drake/multibody/ik_trajectory_helper.h"

#include <cmath>
#include <set>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace plants {

IKTrajectoryHelper::IKTrajectoryHelper(
    int nq, int nT, const double* t,
    int num_qfree, int num_qdotfree,
    const IKoptions& ikoptions,
    const double* dt, const double* dt_ratio)
    : nq_(nq), nT_(nT), t_(t),
      num_qfree_(num_qfree), num_qdotfree_(num_qdotfree),
      num_inbetween_t_samples_(0) {

  ikoptions.getQ(Q_);
  ikoptions.getQa(Qa_);
  ikoptions.getQv(Qv_);

  // Calculate our velocity and acceleration matrices.  This part can
  // be rewritten using the sparse matrix if efficiency becomes a
  // concern.
  //
  // We can't actually calculate these if we only have two timesteps,
  // so we skip a bunch of work in that case.
  if (nT > 2) {
    MatrixXd velocity_mat1 = MatrixXd::Zero(nq * nT, nq * nT);
    MatrixXd velocity_mat2 = MatrixXd::Zero(nq * nT, nq * nT);
    velocity_mat1.block(0, 0, nq, nq) = MatrixXd::Identity(nq, nq);
    velocity_mat1.block(nq * (nT - 1), nq * (nT - 1), nq, nq) =
        MatrixXd::Identity(nq, nq);
    for (int j = 1; j < nT - 1; j++) {
      double val_tmp1 = dt[j - 1];
      double val_tmp2 = dt[j - 1] * (2.0 + 2.0 * dt_ratio[j - 1]);
      double val_tmp3 = dt[j - 1] * dt_ratio[j - 1];
      double val_tmp4 = 3.0 - 3.0 * dt_ratio[j - 1] * dt_ratio[j - 1];
      double val_tmp5 = 3.0 - val_tmp4;
      for (int k = 0; k < nq; k++) {
        velocity_mat1(j * nq + k, (j - 1) * nq + k) = val_tmp1;
        velocity_mat1(j * nq + k, j * nq + k) = val_tmp2;
        velocity_mat1(j * nq + k, (j + 1) * nq + k) = val_tmp3;
        velocity_mat2(j * nq + k, (j - 1) * nq + k) = -3.0;
        velocity_mat2(j * nq + k, j * nq + k) = val_tmp4;
        velocity_mat2(j * nq + k, (j + 1) * nq + k) = val_tmp5;
      }
    }
    velocity_mat_.resize(nq * (nT - 2), nq * nT);
    velocity_mat_ =
        velocity_mat1.inverse().block(nq, 0, nq * (nT - 2), nq * nT) *
        velocity_mat2;

    MatrixXd velocity_mat1_middle_inv =
        (velocity_mat1.block(nq, nq, nq * (nT - 2), nq * (nT - 2))).inverse();
    velocity_mat_qd0_ = -velocity_mat1_middle_inv *
        velocity_mat1.block(nq, 0, nq * (nT - 2), nq);
    velocity_mat_qdf_ =
        -velocity_mat1_middle_inv *
        velocity_mat1.block(nq, nq * (nT - 1), nq * (nT - 2), nq);
  }

  MatrixXd accel_mat1 = MatrixXd::Zero(nq * nT, nq * nT);
  MatrixXd accel_mat2 = MatrixXd::Zero(nq * nT, nq * nT);
  for (int j = 0; j < nT - 1; j++) {
    double val_tmp1 = -6.0 / (dt[j] * dt[j]);
    double val_tmp2 = -val_tmp1;
    double val_tmp3 = -4.0 / dt[j];
    double val_tmp4 = 0.5 * val_tmp3;
    for (int k = 0; k < nq; k++) {
      accel_mat1(j * nq + k, j * nq + k) = val_tmp1;
      accel_mat1(j * nq + k, (j + 1) * nq + k) = val_tmp2;
      accel_mat2(j * nq + k, j * nq + k) = val_tmp3;
      accel_mat2(j * nq + k, (j + 1) * nq + k) = val_tmp4;
    }
  }
  for (int k = 0; k < nq; k++) {
    double val_tmp1 = 6.0 / (dt[nT - 2] * dt[nT - 2]);
    double val_tmp2 = -val_tmp1;
    double val_tmp3 = 2.0 / dt[nT - 2];
    double val_tmp4 = 4.0 / dt[nT - 2];
    accel_mat1((nT - 1) * nq + k, (nT - 2) * nq + k) = val_tmp1;
    accel_mat1((nT - 1) * nq + k, (nT - 1) * nq + k) = val_tmp2;
    accel_mat2((nT - 1) * nq + k, (nT - 2) * nq + k) = val_tmp3;
    accel_mat2((nT - 1) * nq + k, (nT - 1) * nq + k) = val_tmp4;
  }
  accel_mat_.resize(nq * nT, nq * nT);
  accel_mat_ = accel_mat1;
  if (nT > 2) {
    accel_mat_ +=
        accel_mat2.block(0, nq, nq * nT, nq * (nT - 2)) * velocity_mat_;
  }

  accel_mat_qd0_.resize(nq * nT, nq);
  accel_mat_qd0_ = accel_mat2.block(0, 0, nq * nT, nq);
  if (nT > 2) {
    accel_mat_qd0_ +=
        accel_mat2.block(0, nq, nq * nT, nq * (nT - 2)) * velocity_mat_qd0_;
  }

  accel_mat_qdf_.resize(nq * nT, nq);
  accel_mat_qdf_ = accel_mat2.block(0, (nT - 1) * nq, nT * nq, nq);
  if (nT > 2) {
    accel_mat_qdf_ +=
        accel_mat2.block(0, nq, nq * nT, nq * (nT - 2)) * velocity_mat_qdf_;
  }

  // Calculate all of our "in between" samples.
  std::set<double> t_set(t, t + nT);

  VectorXd inbetween_tSamples;
  inbetween_tSamples.resize(0);
  t_inbetween_.resize(nT - 1);
  for (int i = 0; i < nT - 1; i++) {
    t_inbetween_[i].resize(0);
  }

  {
    Eigen::RowVectorXd inbetween_tSamples_tmp;
    ikoptions.getAdditionaltSamples(inbetween_tSamples_tmp);
    int num_inbetween_tSamples_tmp =
        static_cast<int>(inbetween_tSamples_tmp.cols());
    int knot_idx = 0;
    for (int i = 0; i < num_inbetween_tSamples_tmp; i++) {
      if (inbetween_tSamples_tmp(i) > t[0] &&
          inbetween_tSamples_tmp(i) < t[nT - 1] &&
          t_set.find(inbetween_tSamples_tmp(i)) == t_set.end()) {
        inbetween_tSamples.conservativeResize(inbetween_tSamples.size() + 1);
        inbetween_tSamples(inbetween_tSamples.size() - 1) =
            (inbetween_tSamples_tmp(i));
        {
          while (t[knot_idx + 1] < inbetween_tSamples_tmp(i)) {
            knot_idx++;
          }
          t_inbetween_[knot_idx].conservativeResize(
              t_inbetween_[knot_idx].size() + 1);
          t_inbetween_[knot_idx](t_inbetween_[knot_idx].size() - 1) =
              (inbetween_tSamples_tmp(i) - t[knot_idx]);
        }
      }
    }
  }

  num_inbetween_t_samples_ = static_cast<int>(inbetween_tSamples.size());
  t_samples_.resize(nT + num_inbetween_t_samples_, 0);
  {
    int t_samples_idx = 0;
    for (int i = 0; i < nT - 1; i++) {
      t_samples_[t_samples_idx] = t[i];
      for (int j = 0; j < t_inbetween_[i].size(); j++) {
          t_samples_[t_samples_idx + 1 + j] = t_inbetween_[i](j) + t[i];
      }
      t_samples_idx += 1 + static_cast<int>(t_inbetween_[i].size());
    }
    t_samples_[nT + num_inbetween_t_samples_ - 1] = t[nT - 1];
  }

  // We've created t_inbetween_ and t_samples_.  Now create our
  // per-knot matrices.
  dq_inbetween_dqknot_.resize(nT - 1);
  dq_inbetween_dqd0_.resize(nT - 1);
  dq_inbetween_dqdf_.resize(nT - 1);

  for (int i = 0; i < nT - 1; i++) {
    VectorXd dt_ratio_inbetween_i = t_inbetween_[i] / dt[i];
    int nt_sample_inbetween_i = static_cast<int>(t_inbetween_[i].size());
    dq_inbetween_dqknot_[i] =
        MatrixXd::Zero(nt_sample_inbetween_i * nq, nq * nT);
    dq_inbetween_dqdf_[i] = MatrixXd::Zero(nt_sample_inbetween_i * nq, nq);
    dq_inbetween_dqdf_[i] = MatrixXd::Zero(nt_sample_inbetween_i * nq, nq);
    MatrixXd dq_idqdknot_i = MatrixXd::Zero(nt_sample_inbetween_i * nq, nq);
    MatrixXd dq_idqdknot_ip1 = MatrixXd::Zero(nt_sample_inbetween_i * nq, nq);
    for (int j = 0; j < nt_sample_inbetween_i; j++) {
      double val1 = 1.0 - 3.0 * pow(dt_ratio_inbetween_i[j], 2) +
          2.0 * pow(dt_ratio_inbetween_i[j], 3);
      double val2 = 3.0 * pow(dt_ratio_inbetween_i[j], 2) -
          2.0 * pow(dt_ratio_inbetween_i[j], 3);
      double val3 = (1.0 - 2.0 * dt_ratio_inbetween_i[j] +
                     pow(dt_ratio_inbetween_i[j], 2)) *
          t_inbetween_[i](j);
      double val4 =
          (pow(dt_ratio_inbetween_i[j], 2) - dt_ratio_inbetween_i[j]) *
          t_inbetween_[i](j);
      for (int k = 0; k < nq; k++) {
        dq_inbetween_dqknot_[i](j * nq + k, i * nq + k) = val1;
        dq_inbetween_dqknot_[i](j * nq + k, (i + 1) * nq + k) = val2;
        dq_idqdknot_i(j * nq + k, k) = val3;
        dq_idqdknot_ip1(j * nq + k, k) = val4;
      }
    }

    if (i >= 1 && i <= nT - 3) {
      dq_inbetween_dqknot_[i] +=
          dq_idqdknot_i * velocity_mat_.block(
              (i - 1) * nq, 0, nq, nq * nT) +
          dq_idqdknot_ip1 * velocity_mat_.block(
              i * nq, 0, nq, nq * nT);
      dq_inbetween_dqd0_[i] =
          dq_idqdknot_i * velocity_mat_qd0_.block(
              (i - 1) * nq, 0, nq, nq) +
          dq_idqdknot_ip1 * velocity_mat_qd0_.block(
              i * nq, 0, nq, nq);
      dq_inbetween_dqdf_[i] =
          dq_idqdknot_i * velocity_mat_qdf_.block(
              (i - 1) * nq, 0, nq, nq) +
          dq_idqdknot_ip1 * velocity_mat_qdf_.block(
              i * nq, 0, nq, nq);
    } else if (i == 0 && i != nT - 2) {
      dq_inbetween_dqknot_[i] +=
          dq_idqdknot_ip1 * velocity_mat_.block(
              i * nq, 0, nq, nq * nT);
      dq_inbetween_dqd0_[i] =
          dq_idqdknot_i +
          dq_idqdknot_ip1 * velocity_mat_qd0_.block(
              i * nq, 0, nq, nq);
      dq_inbetween_dqdf_[i] =
          dq_idqdknot_ip1 * velocity_mat_qdf_.block(
              i * nq, 0, nq, nq);
    } else if (i == nT - 2 && i != 0) {
      dq_inbetween_dqknot_[i] +=
          dq_idqdknot_i * velocity_mat_.block(
              (i - 1) * nq, 0, nq, nq * nT);
      dq_inbetween_dqd0_[i] =
          dq_idqdknot_i * velocity_mat_qd0_.block(
              (i - 1) * nq, 0, nq, nq);
      dq_inbetween_dqdf_[i] =
          dq_idqdknot_i * velocity_mat_qdf_.block(
              (i - 1) * nq, 0, nq, nq) +
          dq_idqdknot_ip1;
    } else if (i == 0 && i == nT - 2) {
      dq_inbetween_dqd0_[i] = dq_idqdknot_i;
      dq_inbetween_dqdf_[i] = dq_idqdknot_ip1;
    }
  }
}

double IKTrajectoryHelper::CalculateCost(
    const Eigen::MatrixXd& q,  const Eigen::MatrixXd& q_nom,
    int qstart_idx, const Eigen::VectorXd& qdot0,
    const Eigen::VectorXd& qdotf, bool fix_initial_state,
    Eigen::MatrixXd* dJ_vec) const {
  MatrixXd q_local = q;
  q_local.resize(nq_ * nT_, 1);

  *dJ_vec = MatrixXd::Zero(1, nq_ * (num_qfree_ + num_qdotfree_));
  MatrixXd qdot(nq_ * nT_, 1);
  MatrixXd qddot(nq_ * nT_, 1);
  qdot.block(0, 0, nq_, 1) = qdot0;
  if (nT_ > 2) {
    qdot.block(nq_, 0, nq_ * (nT_ - 2), 1) =
        velocity_mat_ * q_local +
        velocity_mat_qd0_ * qdot0 +
        velocity_mat_qdf_ * qdotf;
  }
  qdot.block(nq_ * (nT_ - 1), 0, nq_, 1) = qdotf;
  qddot = accel_mat_ * q_local +
      accel_mat_qd0_ * qdot0 +
      accel_mat_qdf_ * qdotf;
  q_local.resize(nq_, nT_);
  qdot.resize(nq_, nT_);
  qddot.resize(nq_, nT_);
  MatrixXd q_diff = q_local.block(0, qstart_idx, nq_, num_qfree_) -
                    q_nom.block(0, qstart_idx, nq_, num_qfree_);
  MatrixXd tmp1 = 0.5 * Qa_ * qddot;
  MatrixXd tmp2 = tmp1.cwiseProduct(qddot);
  double J = tmp2.sum();
  MatrixXd tmp3 = 0.5 * Qv_ * qdot;
  MatrixXd tmp4 = tmp3.cwiseProduct(qdot);
  J += tmp4.sum();
  MatrixXd tmp5 = 0.5 * Q_ * q_diff;
  MatrixXd tmp6 = tmp5.cwiseProduct(q_diff);
  J += tmp6.sum();
  MatrixXd dJdqd;
  if (nT_ > 2) {
    // [dJdqd(2) dJdqd(3) dJdqd(nT_-1)]
    dJdqd = 2 * tmp3.block(0, 1, nq_, nT_ - 2);
    dJdqd.resize(1, nq_ * (nT_ - 2));
    dJ_vec->block(0, 0, 1, nq_ * num_qfree_) =
        dJdqd *
        velocity_mat_.block(0, nq_ * qstart_idx,
                            (nT_ - 2) * nq_, nq_ * num_qfree_);
  }
  MatrixXd dJdqdiff = 2 * tmp5;
  dJdqdiff.resize(1, nq_ * num_qfree_);
  dJ_vec->block(0, 0, 1, nq_ * num_qfree_) += dJdqdiff;
  MatrixXd dJdqdd = 2.0 * tmp1;
  dJdqdd.resize(1, nq_ * nT_);
  dJ_vec->block(0, 0, 1, nq_ * num_qfree_) +=
      dJdqdd * accel_mat_.block(0, nq_ * qstart_idx,
                                nq_ * nT_, nq_ * num_qfree_);
  MatrixXd dJdqdotf;
  dJdqdotf = dJdqdd * accel_mat_qdf_ + qdotf.transpose() * Qv_;
  if (nT_ > 2) {
    dJdqdotf += dJdqd * velocity_mat_qdf_;
  }
  dJdqdotf.resize(1, nq_);
  if (fix_initial_state) {
    dJ_vec->block(0, nq_ * num_qfree_, 1, nq_) = dJdqdotf;
  } else {
    MatrixXd dJdqdot0;
    dJdqdot0 = dJdqdd * accel_mat_qd0_ + qdot0.transpose() * Qv_;
    if (nT_ > 2) {
      dJdqdot0 += dJdqd * velocity_mat_qd0_;
    }
    dJdqdot0.resize(1, nq_);
    dJ_vec->block(0, nq_ * num_qfree_, 1, nq_) = dJdqdot0;
    dJ_vec->block(0, nq_ * num_qfree_ + nq_, 1, nq_) = dJdqdotf;
  }

  return J;
}


}  // namespace plants
}  // namespace systems
}  // namespace drake
