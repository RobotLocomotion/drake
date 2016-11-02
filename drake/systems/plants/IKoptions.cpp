#include <set>

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/IKoptions.h"

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::SelfAdjointEigenSolver;
using Eigen::VectorXd;
using std::cerr;
using std::endl;
using std::set;

IKoptions::IKoptions(RigidBodyTree *robot) {
  // It is important to make sure these default values are consistent with the
  // MATLAB IKoptions
  setDefaultParams(robot);
}

IKoptions::IKoptions(const IKoptions &rhs) {
  robot_ = rhs.robot_;
  nq_ = rhs.nq_;
  Q_ = rhs.Q_;
  Qa_ = rhs.Qa_;
  Qv_ = rhs.Qv_;
  debug_mode_ = rhs.debug_mode_;
  sequentialSeedFlag_ = rhs.sequentialSeedFlag_;
  SNOPT_MajorFeasibilityTolerance_ = rhs.SNOPT_MajorFeasibilityTolerance_;
  SNOPT_MajorIterationsLimit_ = rhs.SNOPT_MajorIterationsLimit_;
  SNOPT_IterationsLimit_ = rhs.SNOPT_IterationsLimit_;
  SNOPT_SuperbasicsLimit_ = rhs.SNOPT_SuperbasicsLimit_;
  SNOPT_MajorOptimalityTolerance_ = rhs.SNOPT_MajorOptimalityTolerance_;
  additional_tSamples_ = rhs.additional_tSamples_;
  fixInitialState_ = rhs.fixInitialState_;
  q0_lb_ = rhs.q0_lb_;
  q0_ub_ = rhs.q0_ub_;
  qd0_lb_ = rhs.qd0_lb_;
  qd0_ub_ = rhs.qd0_ub_;
  qdf_lb_ = rhs.qdf_lb_;
  qdf_ub_ = rhs.qdf_ub_;
}
IKoptions::~IKoptions() {}

void IKoptions::setDefaultParams(RigidBodyTree *robot) {
  robot_ = robot;
  nq_ = robot->get_num_positions();
  Q_ = MatrixXd::Identity(nq_, nq_);
  Qa_ = 0.1 * MatrixXd::Identity(nq_, nq_);
  Qv_ = MatrixXd::Zero(nq_, nq_);
  debug_mode_ = true;
  sequentialSeedFlag_ = false;
  SNOPT_MajorFeasibilityTolerance_ = 1E-6;
  SNOPT_MajorIterationsLimit_ = 200;
  SNOPT_IterationsLimit_ = 10000;
  SNOPT_SuperbasicsLimit_ = 2000;
  SNOPT_MajorOptimalityTolerance_ = 1E-4;
  additional_tSamples_.resize(0);
  fixInitialState_ = true;
  q0_lb_ = robot->joint_limit_min;
  q0_ub_ = robot->joint_limit_max;
  qd0_ub_ = VectorXd::Zero(nq_);
  qd0_lb_ = VectorXd::Zero(nq_);
  qdf_ub_ = VectorXd::Zero(nq_);
  qdf_lb_ = VectorXd::Zero(nq_);
}

RigidBodyTree *IKoptions::getRobotPtr() const { return robot_; }

void IKoptions::setQ(const MatrixXd &Q) {
  if (Q.rows() != nq_ || Q.cols() != nq_) {
    cerr << "Q should be nq x nq matrix" << endl;
  }
  Q_ = (Q + Q.transpose()) / 2;
  SelfAdjointEigenSolver<MatrixXd> eigensolver(Q_);
  VectorXd ev = eigensolver.eigenvalues();
  for (int i = 0; i < nq_; i++) {
    if (ev(i) < 0) {
      cerr << "Q is not positive semi-definite" << endl;
    }
  }
}

void IKoptions::setQa(const MatrixXd &Qa) {
  if (Qa.rows() != nq_ || Qa.cols() != nq_) {
    cerr << "Qa should be nq x nq matrix" << endl;
  }
  Qa_ = (Qa + Qa.transpose()) / 2;
  SelfAdjointEigenSolver<MatrixXd> eigensolver(Qa_);
  VectorXd ev = eigensolver.eigenvalues();
  for (int i = 0; i < nq_; i++) {
    if (ev(i) < 0) {
      cerr << "Qa is not positive semi-definite" << endl;
    }
  }
}

void IKoptions::setQv(const MatrixXd &Qv) {
  if (Qv.rows() != nq_ || Qv.cols() != nq_) {
    cerr << "Qv should be nq x nq matrix" << endl;
  }
  Qv_ = (Qv + Qv.transpose()) / 2;
  SelfAdjointEigenSolver<MatrixXd> eigensolver(Qv_);
  VectorXd ev = eigensolver.eigenvalues();
  for (int i = 0; i < nq_; i++) {
    if (ev(i) < 0) {
      cerr << "Qv is not positive semi-definite" << endl;
    }
  }
}
void IKoptions::getQ(MatrixXd &Q) const { Q = Q_; }

void IKoptions::getQa(MatrixXd &Qa) const { Qa = Qa_; }

void IKoptions::getQv(MatrixXd &Qv) const { Qv = Qv_; }

void IKoptions::setDebug(bool flag) { debug_mode_ = flag; }

bool IKoptions::getDebug() const { return debug_mode_; }

void IKoptions::setSequentialSeedFlag(bool flag) {
  sequentialSeedFlag_ = flag;
}

bool IKoptions::getSequentialSeedFlag() const {
  return sequentialSeedFlag_;
}

void IKoptions::setMajorOptimalityTolerance(double tol) {
  if (tol <= 0) {
    cerr << "Major Optimality Tolerance must be positive" << endl;
  }
  SNOPT_MajorOptimalityTolerance_ = tol;
}

double IKoptions::getMajorOptimalityTolerance() const {
  return SNOPT_MajorOptimalityTolerance_;
}

void IKoptions::setMajorFeasibilityTolerance(double tol) {
  if (tol <= 0) {
    cerr << "Major Feasibility Tolerance must be positive" << endl;
  }
  SNOPT_MajorFeasibilityTolerance_ = tol;
}

double IKoptions::getMajorFeasibilityTolerance() const {
  return SNOPT_MajorFeasibilityTolerance_;
}

void IKoptions::setSuperbasicsLimit(int limit) {
  if (limit <= 0) {
    cerr << "Superbasics limit must be positive" << endl;
  }
  SNOPT_SuperbasicsLimit_ = limit;
}

int IKoptions::getSuperbasicsLimit() const {
  return SNOPT_SuperbasicsLimit_;
}

void IKoptions::setMajorIterationsLimit(int limit) {
  if (limit <= 0) {
    cerr << "Major iterations limit must be positive" << endl;
  }
  SNOPT_MajorIterationsLimit_ = limit;
}

int IKoptions::getMajorIterationsLimit() const {
  return SNOPT_MajorIterationsLimit_;
}

void IKoptions::setIterationsLimit(int limit) {
  if (limit <= 0) {
    cerr << "Iterations limit must be positive" << endl;
  }
  SNOPT_IterationsLimit_ = limit;
}

int IKoptions::getIterationsLimit() const {
  return SNOPT_IterationsLimit_;
}

void IKoptions::setFixInitialState(bool flag) { fixInitialState_ = flag; }

bool IKoptions::getFixInitialState() const { return fixInitialState_; }

void IKoptions::setq0(const VectorXd &lb, const VectorXd &ub) {
  if (lb.rows() != nq_ || ub.rows() != nq_) {
    cerr << "q0_lb and q0_ub must be nq x 1 column vector" << endl;
  }
  q0_lb_ = lb;
  q0_ub_ = ub;
  for (int i = 0; i < nq_; i++) {
    if (q0_lb_(i) > q0_ub_(i)) {
      cerr << "q0_lb must be no larger than q0_ub" << endl;
    }
    q0_lb_(i) = q0_lb_(i) > robot_->joint_limit_min[i]
                ? q0_lb_(i)
                : robot_->joint_limit_min[i];
    q0_ub_(i) = q0_ub_(i) < robot_->joint_limit_max[i]
                ? q0_ub_(i)
                : robot_->joint_limit_max[i];
  }
}

void IKoptions::getq0(VectorXd &lb, VectorXd &ub) const {
  lb = q0_lb_;
  ub = q0_ub_;
}

void IKoptions::setqd0(const VectorXd &lb, const VectorXd &ub) {
  if (lb.rows() != nq_ || ub.rows() != nq_) {
    cerr << "qd0_lb and qd0_ub must be nq x 1 column vector" << endl;
  }
  for (int i = 0; i < nq_; i++) {
    if (lb(i) > ub(i)) {
      cerr << "qd0_lb must be no larger than qd0_ub" << endl;
    }
  }
  qd0_lb_ = lb;
  qd0_ub_ = ub;
}

void IKoptions::getqd0(VectorXd &lb, VectorXd &ub) const {
  lb = qd0_lb_;
  ub = qd0_ub_;
}

void IKoptions::setqdf(const VectorXd &lb, const VectorXd &ub) {
  if (lb.rows() != nq_ || ub.rows() != nq_) {
    cerr << "qdf_lb and qdf_ub must be nq x 1 column vector" << endl;
  }
  for (int i = 0; i < nq_; i++) {
    if (lb(i) > ub(i)) {
      cerr << "qdf_lb must be no larger than qdf_ub" << endl;
    }
  }
  qdf_lb_ = lb;
  qdf_ub_ = ub;
}

void IKoptions::getqdf(VectorXd &lb, VectorXd &ub) const {
  lb = qdf_lb_;
  ub = qdf_ub_;
}

void IKoptions::setAdditionaltSamples(const RowVectorXd &t_samples) {
  if (t_samples.size() > 0) {
    set<double> unique_sort_t(t_samples.data(),
                              t_samples.data() + t_samples.size());
    additional_tSamples_.resize(unique_sort_t.size());
    int t_idx = 0;
    for (auto it = unique_sort_t.begin(); it != unique_sort_t.end(); it++) {
      additional_tSamples_(t_idx) = *it;
      t_idx++;
    }
  } else {
    additional_tSamples_.resize(0);
  }
}

void IKoptions::getAdditionaltSamples(RowVectorXd &t_samples) const {
  t_samples = additional_tSamples_;
}

void IKoptions::updateRobot(RigidBodyTree *new_robot) {
  robot_ = new_robot;
  int nq_cache = nq_;
  nq_ = robot_->get_num_positions();
  if (nq_cache != nq_) {
    setDefaultParams(new_robot);
  }
}
