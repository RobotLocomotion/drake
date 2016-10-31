#include "drake/systems/plants/constraint/RigidBodyConstraint.h"

// TODO(#2274) NOTE This file has so many cpplint errors that we have
// whitelisted it in its entirety.  When the file is next rewritten or updates,
// we should re-enable cpplint accordingly.

#include <map>
#include <stdexcept>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/math/quaternion.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::AutoDiffScalar;
using Eigen::Isometry3d;
using Eigen::Map;
using Eigen::Matrix3Xd;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::math::initializeAutoDiffTuple;
using drake::math::quatDiff;
using drake::math::quatDiffAxisInvar;

namespace DrakeRigidBodyConstraint {
Vector2d default_tspan(-std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::infinity());
}

RigidBodyConstraint::RigidBodyConstraint(int category, RigidBodyTree* robot,
                                         const Vector2d& tspan)
    : category_(category), type_(0), robot_(robot) {
  if (category >= 0 || category <= -7) {
    throw std::runtime_error(
        "Drake:RigidBodyConstraint:Unsupported constraint category");
  }
  if (tspan(0) > tspan(1)) {
    throw std::runtime_error(
        "Drake:RigidBodyConstraint:tspan(0) should be no larger than tspan(1)");
  }
  tspan_[0] = tspan(0);
  tspan_[1] = tspan(1);
}

RigidBodyConstraint::~RigidBodyConstraint(void) {}

std::string RigidBodyConstraint::getTimeString(const double* t) const {
  std::string time_str;
  if (t != nullptr) {
    time_str = " at time " + std::to_string(*t);
  }
  return time_str;
}

namespace {
const int QuasiStaticDefaultRobotNum[1] = {0};
}

const std::set<int> QuasiStaticConstraint::defaultRobotNumSet(
    QuasiStaticDefaultRobotNum, QuasiStaticDefaultRobotNum + 1);
QuasiStaticConstraint::QuasiStaticConstraint(RigidBodyTree* robot,
    const Vector2d& tspan, const std::set<int>& model_instance_id_set)
    : RigidBodyConstraint(RigidBodyConstraint::QuasiStaticConstraintCategory,
                          robot, tspan),
      m_model_instance_id_set_(model_instance_id_set),
      shrink_factor_(0.9),
      active_(false),
      num_bodies_(0),
      num_pts_(0) {
  set_type(RigidBodyConstraint::QuasiStaticConstraintType);
}

QuasiStaticConstraint::~QuasiStaticConstraint() {}

bool QuasiStaticConstraint::isTimeValid(const double* t) const {
  if (t == nullptr) return true;
  return (*t) >= tspan()[0] && (*t) <= tspan()[1];
}

int QuasiStaticConstraint::getNumConstraint(const double* t) const {
  if (isTimeValid(t)) {
    return 3;
  } else {
    return 0;
  }
}

void QuasiStaticConstraint::updateRobot(RigidBodyTree* robot) {
  set_robot(robot);
}
void QuasiStaticConstraint::eval(const double* t,
                                 KinematicsCache<double>& cache,
                                 const double* weights, VectorXd& c,
                                 MatrixXd& dc) const {
  if (isTimeValid(t)) {
    int nq = getRobotPointer()->get_num_positions();
    dc.resize(2, nq + num_pts_);
    auto com = getRobotPointer()->centerOfMass(cache, m_model_instance_id_set_);
    auto dcom =
        getRobotPointer()->centerOfMassJacobian(cache, m_model_instance_id_set_,
            true);
    MatrixXd contact_pos(3, num_pts_);
    MatrixXd dcontact_pos(3 * num_pts_, nq);
    int num_accum_pts = 0;
    Vector3d center_pos = Vector3d::Zero();
    MatrixXd dcenter_pos = MatrixXd::Zero(3, nq);
    for (int i = 0; i < num_bodies_; i++) {
      auto body_contact_pos = getRobotPointer()->transformPoints(
          cache, body_pts_[i], bodies_[i], 0);
      auto dbody_contact_pos = getRobotPointer()->transformPointsJacobian(
          cache, body_pts_[i], bodies_[i], 0, true);

      contact_pos.block(0, num_accum_pts, 3, num_body_pts_[i]) =
          body_contact_pos;
      dcontact_pos.block(3 * num_accum_pts, 0, 3 * num_body_pts_[i], nq) =
          dbody_contact_pos;
      for (int j = 0; j < num_body_pts_[i]; j++) {
        center_pos = center_pos + body_contact_pos.col(j);
        dcenter_pos = dcenter_pos + dbody_contact_pos.block(3 * j, 0, 3, nq);
      }
      num_accum_pts += num_body_pts_[i];
    }
    center_pos = center_pos / num_pts_;
    dcenter_pos = dcenter_pos / num_pts_;
    MatrixXd support_pos(2, num_pts_);
    MatrixXd dsupport_pos(2 * num_pts_, nq);
    c = com.head(2);
    dc.block(0, 0, 2, nq) = dcom.block(0, 0, 2, nq);
    for (int i = 0; i < num_pts_; i++) {
      support_pos.col(i) = center_pos.head(2) * (1.0 - shrink_factor_) +
                           contact_pos.block(0, i, 2, 1) * shrink_factor_;
      dsupport_pos.block(2 * i, 0, 2, nq) =
          dcenter_pos.block(0, 0, 2, nq) * (1.0 - shrink_factor_) +
          dcontact_pos.block(3 * i, 0, 2, nq) * shrink_factor_;
      c = c - weights[i] * support_pos.col(i);
      dc.block(0, 0, 2, nq) = dc.block(0, 0, 2, nq) -
                              weights[i] * dsupport_pos.block(2 * i, 0, 2, nq);
    }
    dc.block(0, nq, 2, num_pts_) = -support_pos;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void QuasiStaticConstraint::bounds(const double* t, VectorXd& lb,
                                   VectorXd& ub) const {
  if (isTimeValid(t)) {
    lb.resize(2);
    ub.resize(2);
    lb << 0.0, 0.0;
    ub << 0.0, 0.0;
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}

void QuasiStaticConstraint::name(const double* t,
                                 std::vector<std::string>& name_str) const {
  std::string time_str = getTimeString(t);
  name_str.push_back("QuasiStaticConstraint x" + time_str);
  name_str.push_back("QuasiStaticConstraint y" + time_str);
}

static bool compare3Dvector(const Vector3d& a, const Vector3d& b) {
  if (a(0) < b(0)) return true;
  if (a(0) > b(0)) return false;
  if (a(1) < b(1)) return true;
  if (a(1) > b(1)) return false;
  if (a(2) < b(2)) return true;
  return false;
}
void QuasiStaticConstraint::addContact(int num_new_bodies,
                                       const int* new_bodies,
                                       const Matrix3Xd* new_body_pts) {
  for (int i = 0; i < num_new_bodies; i++) {
    bool findDuplicateBody = false;
    for (int j = 0; j < num_bodies_; j++) {
      if (bodies_[j] == new_bodies[i]) {
        findDuplicateBody = true;
        bool (*compare3Dvector_ptr)(const Vector3d&, const Vector3d&) =
            compare3Dvector;
        std::set<Vector3d, bool (*)(const Vector3d&, const Vector3d&)>
            unique_body_pts(compare3Dvector_ptr);
        for (int k = 0; k < body_pts_[j].cols(); k++) {
          unique_body_pts.insert(body_pts_[j].block(0, k, 3, 1));
        }
        for (int k = 0; k < new_body_pts[i].cols(); k++) {
          unique_body_pts.insert(new_body_pts[i].block(0, k, 3, 1));
        }
        num_pts_ -= num_body_pts_[j];
        num_body_pts_[j] = static_cast<int>(unique_body_pts.size());
        num_pts_ += num_body_pts_[j];
        body_pts_[j].resize(3, num_body_pts_[j]);
        int col_idx = 0;
        for (auto it = unique_body_pts.begin(); it != unique_body_pts.end();
             it++) {
          body_pts_[j].block(0, col_idx, 3, 1) = *it;
          col_idx++;
        }
      }
    }
    if (!findDuplicateBody) {
      bodies_.push_back(new_bodies[i]);
      num_body_pts_.push_back(static_cast<int>(new_body_pts[i].cols()));
      body_pts_.push_back(new_body_pts[i]);
      num_bodies_++;
      num_pts_ += static_cast<int>(new_body_pts[i].cols());
    }
  }
}

void QuasiStaticConstraint::setShrinkFactor(double factor) {
  if (factor <= 0.0) {
    throw std::runtime_error("factor should be positive");
  }
  shrink_factor_ = factor;
}

void QuasiStaticConstraint::updateRobotnum(std::set<int>&
    model_instance_id_set) {
  m_model_instance_id_set_ = model_instance_id_set;
}

PostureConstraint::PostureConstraint(RigidBodyTree* robot,
                                     const Eigen::Vector2d& tspan)
    : RigidBodyConstraint(RigidBodyConstraint::PostureConstraintCategory, robot,
                          tspan),
      joint_limit_min0_(robot->joint_limit_min),
      joint_limit_max0_(robot->joint_limit_max),
      lb_(joint_limit_min0_),
      ub_(joint_limit_max0_) {
  set_type(RigidBodyConstraint::PostureConstraintType);
}

bool PostureConstraint::isTimeValid(const double* t) const {
  if (t == nullptr) return true;
  return (*t) >= tspan()[0] && (*t) <= tspan()[1];
}

void PostureConstraint::setJointLimits(const VectorXi& joint_idx,
                                       const VectorXd& lb, const VectorXd& ub) {
  return setJointLimits(joint_idx.size(), joint_idx.data(), lb, ub);
}

void PostureConstraint::setJointLimits(int num_idx, const int* joint_idx,
                                       const VectorXd& lb, const VectorXd& ub) {
  for (int i = 0; i < num_idx; i++) {
    if (joint_idx[i] >= getRobotPointer()->get_num_positions() ||
        joint_idx[i] < 0) {
      throw std::runtime_error("all joint_idx should be within [0 nq-1]");
    }
    if (lb[i] > getRobotPointer()->joint_limit_max[joint_idx[i]]) {
      throw std::runtime_error(
          "joint lb is greater than the robot default joint maximum");
    }
    if (ub[i] < getRobotPointer()->joint_limit_min[joint_idx[i]]) {
      throw std::runtime_error(
          "joint ub is smaller than the robot default joint minimum");
    }
    lb_[joint_idx[i]] =
        (getRobotPointer()->joint_limit_min[joint_idx[i]] < lb[i]
             ? lb[i]
             : getRobotPointer()->joint_limit_min[joint_idx[i]]);
    ub_[joint_idx[i]] =
        (getRobotPointer()->joint_limit_max[joint_idx[i]] > ub[i]
             ? ub[i]
             : getRobotPointer()->joint_limit_max[joint_idx[i]]);
  }
}

void PostureConstraint::bounds(const double* t, VectorXd& joint_min,
                               VectorXd& joint_max) const {
  if (isTimeValid(t)) {
    joint_min = lb_;
    joint_max = ub_;
  } else {
    joint_min = getRobotPointer()->joint_limit_min;
    joint_max = getRobotPointer()->joint_limit_max;
  }
}

MultipleTimeLinearPostureConstraint::MultipleTimeLinearPostureConstraint(
    RigidBodyTree* robot, const Eigen::Vector2d& tspan)
    : RigidBodyConstraint(
          RigidBodyConstraint::MultipleTimeLinearPostureConstraintCategory,
          robot, tspan) {}

std::vector<bool> MultipleTimeLinearPostureConstraint::isTimeValid(
    const double* t, int n_breaks) const {
  std::vector<bool> flag;
  for (int i = 0; i < n_breaks; i++) {
    if (i < n_breaks - 1) {
      if (t[i + 1] < t[i]) {
        throw std::runtime_error("t must be in ascending order");
      }
    }
    if ((t[i] > tspan()[1] || t[i] < tspan()[0])) {
      flag.push_back(false);
    } else {
      flag.push_back(true);
    }
  }
  return flag;
}

int MultipleTimeLinearPostureConstraint::numValidTime(
    const std::vector<bool>& valid_flag) const {
  int num_valid_t = 0;
  for (auto it = valid_flag.begin(); it != valid_flag.end(); it++) {
    if (*it) {
      num_valid_t++;
    }
  }
  return num_valid_t;
}

void MultipleTimeLinearPostureConstraint::validTimeInd(
    const std::vector<bool>& valid_flag, VectorXi& valid_t_ind) const {
  valid_t_ind.resize(0);
  for (int i = 0; i < static_cast<int>(valid_flag.size()); i++) {
    if (valid_flag.at(i)) {
      valid_t_ind.conservativeResize(valid_t_ind.size() + 1);
      valid_t_ind(valid_t_ind.size() - 1) = i;
    }
  }
}

void MultipleTimeLinearPostureConstraint::eval(const double* t, int n_breaks,
                                               const MatrixXd& q, VectorXd& c,
                                               SparseMatrix<double>& dc) const {
  feval(t, n_breaks, q, c);
  VectorXi iAfun;
  VectorXi jAvar;
  VectorXd A;
  geval(t, n_breaks, iAfun, jAvar, A);
  int num_cnst = getNumConstraint(t, n_breaks);
  dc.resize(num_cnst, static_cast<int>(q.size()));
  dc.reserve(static_cast<int>(A.size()));
  for (int i = 0; i < iAfun.size(); i++) {
    dc.insert(iAfun(i), jAvar(i)) = A(i);
  }
}

SingleTimeLinearPostureConstraint::SingleTimeLinearPostureConstraint(
    RigidBodyTree* robot, const VectorXi& iAfun, const VectorXi& jAvar,
    const VectorXd& A, const VectorXd& lb, const VectorXd& ub,
    const Vector2d& tspan)
    : RigidBodyConstraint(
          RigidBodyConstraint::SingleTimeLinearPostureConstraintCategory, robot,
          tspan),
      lb_(lb),
      ub_(ub),
      num_constraint_(static_cast<int>(lb_.size())) {
  for (int i = 0; i < num_constraint_; i++) {
    if (lb(i) > ub(i)) {
      throw std::runtime_error(
          "SingleTimeLinearPostureConstraint: lb must be no larger than ub");
    }
  }
  int lenA = static_cast<int>(iAfun.size());
  if (jAvar.size() != lenA || A.size() != lenA) {
    throw std::runtime_error(
        "SingleTimeLinearPostureConstraint: "
        "iAfun, jAvar and A should be of the same size");
  }
  std::set<std::pair<int, int>> mat_ind_set;
  for (int i = 0; i < lenA; i++) {
    std::pair<int, int> ind_pair(iAfun(i), jAvar(i));
    mat_ind_set.insert(ind_pair);
  }
  if (static_cast<int>(mat_ind_set.size()) != lenA) {
    throw std::runtime_error(
        "SingleTimeLinearPostureConstraint: "
        "The pair (iAfun(i), jAvar(i)) is not unique");
  }
  if (iAfun.maxCoeff() != num_constraint_ - 1) {
    throw std::runtime_error(
        "SingleTimeLinearPostureConstraint: "
        "max(iAfun) should be lb.size() - 1");
  }
  if (iAfun.minCoeff() != 0) {
    throw std::runtime_error(
        "SingleTimeLinearPostureConstraint: min(iAfun) should be 0");
  }
  if (jAvar.minCoeff() < 0 ||
      jAvar.maxCoeff() > getRobotPointer()->get_num_positions() - 1) {
    throw std::runtime_error(
        "SingleTimeLinearPostureConstraint: "
        "jAvar should be within[0 robot.nq-1]");
  }
  iAfun_ = iAfun;
  jAvar_ = jAvar;
  A_ = A;
  A_mat_.resize(num_constraint_,
                getRobotPointer()->get_num_positions());
  A_mat_.reserve(lenA);

  for (int i = 0; i < lenA; i++) {
    A_mat_.insert(iAfun(i), jAvar(i)) = A(i);
  }
  set_type(RigidBodyConstraint::SingleTimeLinearPostureConstraintType);
}

bool SingleTimeLinearPostureConstraint::isTimeValid(const double* t) const {
  if (t == nullptr) {
    return true;
  }
  if (*t >= tspan()[0] && *t <= tspan()[1]) {
    return true;
  }
  return false;
}

int SingleTimeLinearPostureConstraint::getNumConstraint(const double* t) const {
  if (isTimeValid(t)) {
    return num_constraint_;
  } else {
    return 0;
  }
}

void SingleTimeLinearPostureConstraint::bounds(const double* t, VectorXd& lb,
                                               VectorXd& ub) const {
  if (isTimeValid(t)) {
    lb = lb_;
    ub = ub_;
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}

void SingleTimeLinearPostureConstraint::feval(const double* t,
                                              const VectorXd& q,
                                              VectorXd& c) const {
  if (isTimeValid(t)) {
    c = A_mat_ * q;
  } else {
    c.resize(0);
  }
}

void SingleTimeLinearPostureConstraint::geval(const double* t, VectorXi& iAfun,
                                              VectorXi& jAvar,
                                              VectorXd& A) const {
  if (isTimeValid(t)) {
    iAfun = iAfun_;
    jAvar = jAvar_;
    A = A_;
  } else {
    iAfun.resize(0);
    jAvar.resize(0);
    A.resize(0);
  }
}

void SingleTimeLinearPostureConstraint::eval(const double* t, const VectorXd& q,
                                             VectorXd& c,
                                             SparseMatrix<double>& dc) const {
  if (isTimeValid(t)) {
    c = A_mat_ * q;
    dc = A_mat_;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void SingleTimeLinearPostureConstraint::name(
    const double* t, std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    for (int i = 0; i < num_constraint_; i++) {
      name_str.push_back("SingleTimeLinearPostureConstraint row " +
                         std::to_string(i) + time_str);
    }
  }
}

SingleTimeKinematicConstraint::SingleTimeKinematicConstraint(
    RigidBodyTree* robot, const Vector2d& tspan)
    : RigidBodyConstraint(
          RigidBodyConstraint::SingleTimeKinematicConstraintCategory, robot,
          tspan),
      num_constraint_(0) {}

bool SingleTimeKinematicConstraint::isTimeValid(const double* t) const {
  if (t == nullptr) return true;
  return (*t) >= tspan()[0] && (*t) <= tspan()[1];
}

int SingleTimeKinematicConstraint::getNumConstraint(const double* t) const {
  if (isTimeValid(t)) {
    return num_constraint_;
  }
  return 0;
}

void SingleTimeKinematicConstraint::updateRobot(RigidBodyTree* robot) {
  set_robot(robot);
}

MultipleTimeKinematicConstraint::MultipleTimeKinematicConstraint(
    RigidBodyTree* robot, const Vector2d& tspan)
    : RigidBodyConstraint(
          RigidBodyConstraint::MultipleTimeKinematicConstraintCategory, robot,
          tspan) {}

void MultipleTimeKinematicConstraint::eval(const double* t, int n_breaks,
                                           const MatrixXd& q, VectorXd& c,
                                           MatrixXd& dc) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2) {
    std::vector<bool> valid_time_flag = isTimeValid(t, n_breaks);
    int nq = getRobotPointer()->get_num_positions();
    double* valid_t = new double[num_valid_t];
    MatrixXd valid_q(nq, num_valid_t);
    int valid_idx = 0;
    int* valid2tMap = new int[num_valid_t];
    for (int i = 0; i < n_breaks; i++) {
      if (valid_time_flag[i]) {
        valid_t[valid_idx] = t[i];
        valid_q.col(valid_idx) = q.col(i);
        valid2tMap[valid_idx] = i;
        valid_idx++;
      }
    }
    MatrixXd dc_valid;
    eval_valid(valid_t, num_valid_t, valid_q, c, dc_valid);
    int nc = getNumConstraint(t, n_breaks);
    dc = MatrixXd::Zero(nc, nq * n_breaks);
    for (int i = 0; i < num_valid_t; i++) {
      dc.block(0, valid2tMap[i] * nq, nc, nq) =
          dc_valid.block(0, i * nq, nc, nq);
    }
    delete[] valid_t;
    delete[] valid2tMap;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

std::vector<bool> MultipleTimeKinematicConstraint::isTimeValid(
    const double* t, int n_breaks) const {
  std::vector<bool> flag;
  for (int i = 0; i < n_breaks; i++) {
    if ((t[i] > tspan()[1] || t[i] < tspan()[0])) {
      flag.push_back(false);
    } else {
      flag.push_back(true);
    }
  }
  return flag;
}

int MultipleTimeKinematicConstraint::numValidTime(const double* t,
                                                  int n_breaks) const {
  std::vector<bool> valid_flag = isTimeValid(t, n_breaks);
  int num_valid_t = 0;
  for (auto it = valid_flag.begin(); it != valid_flag.end(); it++) {
    if (*it) {
      num_valid_t++;
    }
  }
  return num_valid_t;
}

void MultipleTimeKinematicConstraint::updateRobot(RigidBodyTree* robot) {
  set_robot(robot);
}

PositionConstraint::PositionConstraint(RigidBodyTree* robot,
                                       const Matrix3Xd& pts, MatrixXd lb,
                                       MatrixXd ub, const Vector2d& tspan)
    : SingleTimeKinematicConstraint(robot, tspan),
      pts_(pts),
      n_pts_(static_cast<int>(pts.cols())) {
  if (pts_.rows() != 3) {
    throw std::runtime_error("pts must have 3 rows");
  }

  if (lb.rows() != 3 || lb.cols() != n_pts_ || ub.rows() != 3 ||
      ub.cols() != n_pts_) {
    throw std::runtime_error(
        "lb and ub must have 3 rows, the same number of columns as pts");
  }

  null_constraint_rows_.resize(3 * n_pts_);
  int num_constraint = 0;
  for (int j = 0; j < n_pts_; j++) {
    for (int i = 0; i < 3; i++) {
      int idx = j * 3 + i;
      if (std::isnan(lb(i, j))) {
        lb(i, j) = -std::numeric_limits<double>::infinity();
      }
      if (std::isnan(ub(i, j))) {
        ub(i, j) = std::numeric_limits<double>::infinity();
      }
      if (ub(i, j) < lb(i, j)) {
        throw std::runtime_error("lb must be no larger than ub");
      }
      if (std::isinf(lb(i, j)) && std::isinf(ub(i, j))) {
        null_constraint_rows_[idx] = true;
      } else {
        null_constraint_rows_[idx] = false;
        num_constraint++;
      }
    }
  }
  set_num_constraint(num_constraint);
  lb_.resize(num_constraint);
  ub_.resize(num_constraint);
  int valid_row_idx = 0;
  int valid_col_idx = 0;
  int bnd_idx = 0;
  while (bnd_idx < num_constraint) {
    int idx = 3 * valid_col_idx + valid_row_idx;
    if (!null_constraint_rows_[idx]) {
      lb_[bnd_idx] = lb(valid_row_idx, valid_col_idx);
      ub_[bnd_idx] = ub(valid_row_idx, valid_col_idx);
      bnd_idx++;
    }
    valid_row_idx++;
    if (valid_row_idx == 3) {
      valid_row_idx = 0;
      valid_col_idx++;
    }
  }
}

void PositionConstraint::eval(const double* t, KinematicsCache<double>& cache,
                              VectorXd& c, MatrixXd& dc) const {
  if (isTimeValid(t)) {
    Matrix3Xd pos(3, n_pts_);
    MatrixXd J(3 * n_pts_, getRobotPointer()->get_num_positions());
    evalPositions(cache, pos, J);
    c.resize(getNumConstraint(t), 1);
    dc.resize(getNumConstraint(t),
              getRobotPointer()->get_num_positions());
    int valid_row_idx = 0;
    int i = 0;
    while (i < getNumConstraint(t)) {
      if (!null_constraint_rows_[valid_row_idx]) {
        c(i) = pos(valid_row_idx);
        dc.row(i) = J.row(valid_row_idx);
        i++;
        valid_row_idx++;
      } else {
        valid_row_idx++;
      }
    }
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void PositionConstraint::bounds(const double* t, VectorXd& lb,
                                VectorXd& ub) const {
  if (isTimeValid(t)) {
    lb = lb_;
    ub = ub_;
  }
}

void PositionConstraint::name(const double* t,
                              std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::vector<std::string> cnst_names;
    evalNames(t, cnst_names);
    for (int i = 0; i < 3 * n_pts_; i++) {
      if (!null_constraint_rows_[i]) {
        name_str.push_back(cnst_names.at(i));
      }
    }
  }
}

WorldPositionConstraint::WorldPositionConstraint(RigidBodyTree* robot, int body,
                                                 const Matrix3Xd& pts,
                                                 MatrixXd lb, MatrixXd ub,
                                                 const Vector2d& tspan)
    : PositionConstraint(robot, pts, lb, ub, tspan),
      body_(body),
      body_name_(robot->getBodyOrFrameName(body)) {
  set_type(RigidBodyConstraint::WorldPositionConstraintType);
}

WorldPositionConstraint::~WorldPositionConstraint() {}

void WorldPositionConstraint::evalPositions(KinematicsCache<double>& cache,
                                            Matrix3Xd& pos, MatrixXd& J) const {
  pos = getRobotPointer()->transformPoints(cache, get_pts(), body_, 0);
  J = getRobotPointer()->transformPointsJacobian(cache, get_pts(), body_, 0,
                                                 true);
}

void WorldPositionConstraint::evalNames(
    const double* t, std::vector<std::string>& cnst_names) const {
  std::string time_str = getTimeString(t);
  for (int i = 0; i < get_n_pts(); i++) {
    cnst_names.push_back(body_name_ + " pts(:," + std::to_string(i + 1) +
                         ") x" + time_str);
    cnst_names.push_back(body_name_ + " pts(:," + std::to_string(i + 1) +
                         ") y" + time_str);
    cnst_names.push_back(body_name_ + " pts(:," + std::to_string(i + 1) +
                         ") z" + time_str);
  }
}

namespace {
const int WorldCoMDefaultRobotNum[1] = {0};
}

const std::set<int> WorldCoMConstraint::defaultRobotNumSet(
    WorldCoMDefaultRobotNum, WorldCoMDefaultRobotNum + 1);

WorldCoMConstraint::WorldCoMConstraint(
    RigidBodyTree* robot, Vector3d lb, Vector3d ub, const Vector2d& tspan,
    const std::set<int>& model_instance_id_set)
    : PositionConstraint(robot, Vector3d::Zero(), lb, ub, tspan),
      m_model_instance_id_(model_instance_id_set) {
  set_type(RigidBodyConstraint::WorldCoMConstraintType);
}

void WorldCoMConstraint::evalPositions(KinematicsCache<double>& cache,
                                       Matrix3Xd& pos, MatrixXd& J) const {
  pos = getRobotPointer()->centerOfMass(cache, m_model_instance_id_);
  J = getRobotPointer()->centerOfMassJacobian(cache, m_model_instance_id_,
      true);
}

void WorldCoMConstraint::evalNames(const double* t,
                                   std::vector<std::string>& cnst_names)
                                       const {
  std::string time_str = getTimeString(t);
  cnst_names.push_back("CoM x " + time_str);
  cnst_names.push_back("CoM y " + time_str);
  cnst_names.push_back("CoM z " + time_str);
}

void WorldCoMConstraint::updateRobotnum(
    const std::set<int>& model_instance_id_set) {
  m_model_instance_id_ = model_instance_id_set;
}

WorldCoMConstraint::~WorldCoMConstraint() {}

RelativePositionConstraint::RelativePositionConstraint(
    RigidBodyTree* robot, const Matrix3Xd& pts, const MatrixXd& lb,
    const MatrixXd& ub, int bodyA_idx, int bodyB_idx,
    const Matrix<double, 7, 1>& bTbp, const Vector2d& tspan)
    : PositionConstraint(robot, pts, lb, ub, tspan),
      bodyA_idx_(bodyA_idx),
      bodyB_idx_(bodyB_idx),
      bodyA_name_(robot->getBodyOrFrameName(bodyA_idx)),
      bodyB_name_(robot->getBodyOrFrameName(bodyB_idx)) {
  Isometry3d bTbp_isometry;
  bTbp_isometry.translation() = bTbp.topRows<3>();
  bTbp_isometry.linear() = drake::math::quat2rotmat(bTbp.bottomRows<4>());
  bTbp_isometry.makeAffine();
  bpTb_ = bTbp_isometry.inverse();
  set_type(RigidBodyConstraint::RelativePositionConstraintType);
}

void RelativePositionConstraint::evalPositions(KinematicsCache<double>& cache,
                                               Matrix3Xd& pos,
                                               MatrixXd& J) const {
  pos = bpTb_ *
        getRobotPointer()->transformPoints(cache, get_pts(), bodyA_idx_,
                                           bodyB_idx_);
  J = getRobotPointer()->transformPointsJacobian(cache, get_pts(), bodyA_idx_,
                                                 bodyB_idx_, true);
  for (int i = 0; i < pos.cols(); i++) {
    J.middleRows<3>(3 * i) = bpTb_.linear() * J.middleRows<3>(3 * i);
  }
}

void RelativePositionConstraint::evalNames(
    const double* t, std::vector<std::string>& cnst_names) const {
  std::string time_str = getTimeString(t);
  for (int i = 0; i < get_n_pts(); i++) {
    cnst_names.push_back(bodyA_name_ + " pts(:," + std::to_string(i + 1) +
                         ") in " + bodyB_name_ + " x" + time_str);
    cnst_names.push_back(bodyA_name_ + " pts(:," + std::to_string(i + 1) +
                         ") in " + bodyB_name_ + " y" + time_str);
    cnst_names.push_back(bodyA_name_ + " pts(:," + std::to_string(i + 1) +
                         ") in " + bodyB_name_ + " z" + time_str);
  }
}

RelativePositionConstraint::~RelativePositionConstraint() {}

QuatConstraint::QuatConstraint(RigidBodyTree* robot, double tol,
                               const Vector2d& tspan)
    : SingleTimeKinematicConstraint(robot, tspan), tol_(tol) {
  if (tol < 0.0 || tol > M_PI) {
    throw std::runtime_error("tol must be within [0 PI]");
  }
  set_num_constraint(1);
}

void QuatConstraint::eval(const double* t, KinematicsCache<double>& cache,
                          VectorXd& c, MatrixXd& dc) const {
  const int num_constraint = getNumConstraint(t);
  c.resize(num_constraint);
  dc.resize(num_constraint, getRobotPointer()->get_num_positions());
  if (isTimeValid(t)) {
    double prod;
    MatrixXd dprod(1, getRobotPointer()->get_num_positions());
    evalOrientationProduct(cache, prod, dprod);
    c(0) = 2.0 * prod * prod - 1.0;
    dc = 4.0 * prod * dprod;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void QuatConstraint::bounds(const double* t, VectorXd& lb, VectorXd& ub) const {
  lb.resize(getNumConstraint(t));
  ub.resize(getNumConstraint(t));
  if (isTimeValid(t)) {
    lb[0] = cos(tol_);
    ub[0] = 1.0;
  }
}

QuatConstraint::~QuatConstraint(void) {}

WorldQuatConstraint::WorldQuatConstraint(RigidBodyTree* robot, int body,
                                         const Vector4d& quat_des, double tol,
                                         const Vector2d& tspan)
    : QuatConstraint(robot, tol, tspan),
      body_(body),
      body_name_(robot->getBodyOrFrameName(body)) {
  if (quat_des.norm() <= 0) {
    throw std::runtime_error("quat_des must be non-zero");
  }
  quat_des_ = quat_des / quat_des.norm();
  set_type(RigidBodyConstraint::WorldQuatConstraintType);
}

void WorldQuatConstraint::evalOrientationProduct(
    const KinematicsCache<double>& cache, double& prod, MatrixXd& dprod) const {
  auto quat = getRobotPointer()->relativeQuaternion(cache, body_, 0);
  auto J = getRobotPointer()->relativeQuaternionJacobian(cache, body_, 0, true);

  prod = quat.transpose() * quat_des_;
  dprod = quat_des_.transpose() * J;
}

void WorldQuatConstraint::name(const double* t,
                               std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    const int num_constraint = getNumConstraint(t);
    for (int i = 0; i < num_constraint; i++) {
      name_str.push_back(body_name_ + " quaternion constraint" + time_str);
    }
  }
}

WorldQuatConstraint::~WorldQuatConstraint() {}

RelativeQuatConstraint::RelativeQuatConstraint(RigidBodyTree* robot,
                                               int bodyA_idx, int bodyB_idx,
                                               const Vector4d& quat_des,
                                               double tol,
                                               const Vector2d& tspan)
    : QuatConstraint(robot, tol, tspan),
      bodyA_idx_(bodyA_idx),
      bodyB_idx_(bodyB_idx),
      bodyA_name_(robot->bodies[bodyA_idx]->get_name()),
      bodyB_name_(robot->bodies[bodyB_idx]->get_name()) {
  const double quat_norm = quat_des.norm();
  quat_des_ = quat_des / quat_norm;
  set_type(RigidBodyConstraint::RelativeQuatConstraintType);
}

void RelativeQuatConstraint::evalOrientationProduct(
    const KinematicsCache<double>& cache, double& prod, MatrixXd& dprod) const {
  auto quat_a2b =
      getRobotPointer()->relativeQuaternion(cache, bodyA_idx_, bodyB_idx_);
  prod = quat_a2b.dot(quat_des_);

  auto dquat_a2bdq = getRobotPointer()->relativeQuaternionJacobian(
      cache, bodyA_idx_, bodyB_idx_, true);
  dprod = quat_des_.transpose() * dquat_a2bdq;
}

void RelativeQuatConstraint::name(const double* t,
                                  std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    std::string tmp_name = bodyA_name_ + " relative to " + bodyB_name_ +
                           " quaternion constraint" + time_str;
    const int num_constraint = getNumConstraint(t);
    for (int i = 0; i < num_constraint; i++) {
      name_str.push_back(tmp_name);
    }
  }
}

RelativeQuatConstraint::~RelativeQuatConstraint() {}

EulerConstraint::EulerConstraint(RigidBodyTree* robot, const Vector3d& lb,
                                 const Vector3d& ub, const Vector2d& tspan)
    : SingleTimeKinematicConstraint(robot, tspan) {
  int num_constraint = 0;
  Vector3d my_lb = lb, my_ub = ub;
  for (int i = 0; i < 3; i++) {
    if (std::isnan(my_lb(i))) {
      my_lb(i) = -std::numeric_limits<double>::infinity();
    }
    if (std::isnan(my_ub(i))) {
      my_ub(i) = std::numeric_limits<double>::infinity();
    }
    if (my_ub(i) < my_lb(i)) {
      throw std::runtime_error("lb must be no larger than ub");
    }
    if (std::isinf(my_lb(i)) && std::isinf(my_ub(i))) {
      null_constraint_rows_[i] = true;
    } else {
      null_constraint_rows_[i] = false;
      num_constraint++;
    }
  }
  set_num_constraint(num_constraint);
  lb_.resize(num_constraint);
  ub_.resize(num_constraint);
  int valid_row_idx = 0;
  int bnd_idx = 0;
  while (bnd_idx < num_constraint) {
    if (!null_constraint_rows_[valid_row_idx]) {
      lb_[bnd_idx] = my_lb(valid_row_idx);
      ub_[bnd_idx] = my_ub(valid_row_idx);
      bnd_idx++;
      valid_row_idx++;
    } else {
      valid_row_idx++;
    }
  }
  avg_rpy_.resize(num_constraint);
  for (int i = 0; i < num_constraint; i++) {
    avg_rpy_[i] = (lb_[i] + ub_[i]) / 2.0;
  }
}

void EulerConstraint::eval(const double* t, KinematicsCache<double>& cache,
                           VectorXd& c, MatrixXd& dc) const {
  const int n_constraint = getNumConstraint(t);
  if (isTimeValid(t)) {
    Vector3d rpy;
    MatrixXd drpy(3, getRobotPointer()->get_num_positions());
    evalrpy(cache, rpy, drpy);
    c.resize(n_constraint);
    dc.resize(n_constraint, getRobotPointer()->get_num_positions());
    int valid_row_idx = 0;
    int i = 0;
    while (i < n_constraint) {
      if (!null_constraint_rows_[valid_row_idx]) {
        c(i) = rpy(valid_row_idx);
        c(i) = angleDiff(avg_rpy_[i], c(i)) + avg_rpy_[i];
        dc.row(i) = drpy.row(valid_row_idx);
        valid_row_idx++;
        i++;
      } else {
        valid_row_idx++;
      }
    }
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void EulerConstraint::bounds(const double* t, VectorXd& lb,
                             VectorXd& ub) const {
  if (isTimeValid(t)) {
    lb = lb_;
    ub = ub_;
  }
}

WorldEulerConstraint::WorldEulerConstraint(RigidBodyTree* robot, int body,
                                           const Vector3d& lb,
                                           const Vector3d& ub,
                                           const Vector2d& tspan)
    : EulerConstraint(robot, lb, ub, tspan),
      body_(body),
      body_name_(robot->getBodyOrFrameName(body)) {
  set_type(RigidBodyConstraint::WorldEulerConstraintType);
}

void WorldEulerConstraint::evalrpy(const KinematicsCache<double>& cache,
                                   Vector3d& rpy, MatrixXd& J) const {
  rpy = getRobotPointer()->relativeRollPitchYaw(cache, body_, 0);
  J = getRobotPointer()->relativeRollPitchYawJacobian(cache, body_, 0, true);
}

void WorldEulerConstraint::name(const double* t,
                                std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    int constraint_idx = 0;
    if (!null_constraint_row(0)) {
      name_str.push_back(body_name_ + " roll" + time_str);
      constraint_idx++;
    }
    if (!null_constraint_row(1)) {
      name_str.push_back(body_name_ + " pitch" + time_str);
      constraint_idx++;
    }
    if (!null_constraint_row(2)) {
      name_str.push_back(body_name_ + " yaw" + time_str);
      constraint_idx++;
    }
  }
}

WorldEulerConstraint::~WorldEulerConstraint() {}

GazeConstraint::GazeConstraint(RigidBodyTree* robot, const Vector3d& axis,
                               double conethreshold, const Vector2d& tspan)
    : SingleTimeKinematicConstraint(robot, tspan),
      conethreshold_(conethreshold) {
  double len_axis = axis.norm();
  if (len_axis <= 0) {
    throw std::runtime_error("axis must be non-zero");
  }
  axis_ = axis / len_axis;
  if (conethreshold_ < 0.0 || conethreshold_ > M_PI + 1E-10) {
    throw std::runtime_error("conethreshold should be within [0 PI]");
  }
}

GazeOrientConstraint::GazeOrientConstraint(
    RigidBodyTree* robot, const Vector3d& axis, const Vector4d& quat_des,
    double conethreshold, double threshold, const Vector2d& tspan)
    : GazeConstraint(robot, axis, conethreshold, tspan), threshold_(threshold) {
  double len_quat_des = quat_des.norm();
  if (len_quat_des <= 0) {
    throw std::runtime_error("quat_des must be non-zero");
  }
  quat_des_ = quat_des / len_quat_des;
  if (threshold_ < 0.0 || threshold_ > M_PI + 1E-10) {
    throw std::runtime_error("threshold should be within [0 PI]");
  }
  set_num_constraint(2);
}

void GazeOrientConstraint::eval(const double* t, KinematicsCache<double>& cache,
                                VectorXd& c, MatrixXd& dc) const {
  // NOLINTNEXTLINE(build/namespaces): Needed for ADL.
  using namespace std;
  // NOLINTNEXTLINE(build/namespaces): Needed for ADL.
  using namespace drake;

  const int num_constraint = getNumConstraint(t);
  c.resize(num_constraint);
  dc.resize(num_constraint, getRobotPointer()->get_num_positions());
  if (isTimeValid(t)) {
    Vector4d quat;
    int nq = getRobotPointer()->get_num_positions();
    MatrixXd dquat(4, nq);
    evalOrientation(cache, quat, dquat);

    auto axis_err_autodiff_args =
        initializeAutoDiffTuple(quat, quat_des_, get_axis());
    auto e_autodiff = quatDiffAxisInvar(get<0>(axis_err_autodiff_args),
                                        get<1>(axis_err_autodiff_args),
                                        get<2>(axis_err_autodiff_args));
    auto axis_err = e_autodiff.value();
    auto daxis_err = e_autodiff.derivatives().transpose().eval();

    // Note: don't do:
    // MatrixXd daxis_err_dq(nq, 1)
    // daxis_err_dq = daxis_err.block(0, 0, 1, 4) * dquat;
    // due to http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1283
    // (no reason you'd want to do this anyway, but it resulted in an issue
    // while updating Eigen, see #3300.)
    MatrixXd daxis_err_dq = daxis_err.block(0, 0, 1, 4) * dquat;

    auto quat_diff_autodiff_args = initializeAutoDiffTuple(quat, quat_des_);
    auto q_diff_autodiff = quatDiff(get<0>(quat_diff_autodiff_args),
                                    get<1>(quat_diff_autodiff_args));
    auto q_diff = autoDiffToValueMatrix(q_diff_autodiff);
    auto dq_diff = autoDiffToGradientMatrix(q_diff_autodiff);

    MatrixXd dq_diff_dq(4, nq);
    dq_diff_dq = dq_diff.block(0, 0, 4, 4) * dquat;
    c << axis_err, q_diff(0);
    dc.row(0) = daxis_err_dq;
    dc.row(1) = dq_diff_dq.row(0);
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void GazeOrientConstraint::bounds(const double* t, VectorXd& lb,
                                  VectorXd& ub) const {
  lb.resize(getNumConstraint(t));
  ub.resize(getNumConstraint(t));
  if (isTimeValid(t)) {
    lb << cos(get_conethreshold()) - 1.0, cos(threshold_ / 2.0);
    ub << 0, std::numeric_limits<double>::infinity();
  }
}

WorldGazeOrientConstraint::WorldGazeOrientConstraint(
    RigidBodyTree* robot, int body, const Vector3d& axis,
    const Vector4d& quat_des, double conethreshold, double threshold,
    const Vector2d& tspan)
    : GazeOrientConstraint(robot, axis, quat_des, conethreshold, threshold,
                           tspan),
      body_(body),
      body_name_(robot->getBodyOrFrameName(body)) {
  set_type(RigidBodyConstraint::WorldGazeOrientConstraintType);
}

void WorldGazeOrientConstraint::evalOrientation(
    const KinematicsCache<double>& cache, Vector4d& quat,
    MatrixXd& dquat_dq) const {
  quat = getRobotPointer()->relativeQuaternion(cache, body_, 0);
  dquat_dq =
      getRobotPointer()->relativeQuaternionJacobian(cache, body_, 0, true);
}

void WorldGazeOrientConstraint::name(const double* t,
                                     std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    name_str.push_back(body_name_ +
                       " conic gaze orientation constraint at time" + time_str);
    name_str.push_back(body_name_ +
                       " revolute gaze orientation constraint at time" +
                       time_str);
  }
}

GazeDirConstraint::GazeDirConstraint(RigidBodyTree* robot, const Vector3d& axis,
                                     const Vector3d& dir, double conethreshold,
                                     const Vector2d& tspan)
    : GazeConstraint(robot, axis, conethreshold, tspan) {
  double len_dir = dir.norm();
  if (len_dir <= 0) {
    throw std::runtime_error("dir should be non-zero");
  }
  dir_ = dir / len_dir;
  set_num_constraint(1);
}

void GazeDirConstraint::bounds(const double* t, VectorXd& lb,
                               VectorXd& ub) const {
  const int num_constraint = getNumConstraint(t);
  lb.resize(num_constraint);
  ub.resize(num_constraint);
  if (isTimeValid(t)) {
    lb[0] = cos(get_conethreshold()) - 1.0;
    ub[0] = 0.0;
  }
}

WorldGazeDirConstraint::WorldGazeDirConstraint(RigidBodyTree* robot, int body,
                                               const Vector3d& axis,
                                               const Vector3d& dir,
                                               double conethreshold,
                                               const Vector2d& tspan)
    : GazeDirConstraint(robot, axis, dir, conethreshold, tspan),
      body_(body),
      body_name_(robot->getBodyOrFrameName(body)) {
  set_type(RigidBodyConstraint::WorldGazeDirConstraintType);
}

void WorldGazeDirConstraint::eval(const double* t,
                                  KinematicsCache<double>& cache, VectorXd& c,
                                  MatrixXd& dc) const {
  if (isTimeValid(t)) {
    Matrix3Xd body_axis_ends(3, 2);
    body_axis_ends.col(0).setZero();
    body_axis_ends.col(1) = get_axis();
    int nq = getRobotPointer()->get_num_positions();
    auto axis_pos =
        getRobotPointer()->transformPoints(cache, body_axis_ends, body_, 0);
    auto daxis_pos = getRobotPointer()->transformPointsJacobian(
        cache, body_axis_ends, body_, 0, true);
    Vector3d axis_world = axis_pos.col(1) - axis_pos.col(0);
    MatrixXd daxis_world =
        daxis_pos.block(3, 0, 3, nq) - daxis_pos.block(0, 0, 3, nq);
    c.resize(1);
    c(0) = axis_world.dot(get_dir()) - 1.0;
    dc = get_dir().transpose() * daxis_world;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void WorldGazeDirConstraint::name(const double* t,
                                  std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    name_str.push_back(body_name_ + " conic gaze direction constraint" +
                       time_str);
  }
}

GazeTargetConstraint::GazeTargetConstraint(
    RigidBodyTree* robot, const Vector3d& axis, const Vector3d& target,
    const Vector3d& gaze_origin, double conethreshold, const Vector2d& tspan)
    : GazeConstraint(robot, axis, conethreshold, tspan),
      target_(target),
      gaze_origin_(gaze_origin) {
  set_num_constraint(1);
}

void GazeTargetConstraint::bounds(const double* t, VectorXd& lb,
                                  VectorXd& ub) const {
  const int num_constraint = getNumConstraint(t);
  lb.resize(num_constraint);
  ub.resize(num_constraint);
  if (isTimeValid(t)) {
    lb[0] = cos(get_conethreshold()) - 1.0;
    ub[0] = 0.0;
  }
}

WorldGazeTargetConstraint::WorldGazeTargetConstraint(
    RigidBodyTree* robot, int body, const Vector3d& axis,
    const Vector3d& target, const Vector3d& gaze_origin, double conethreshold,
    const Vector2d& tspan)
    : GazeTargetConstraint(robot, axis, target, gaze_origin, conethreshold,
                           tspan),
      body_(body),
      body_name_(robot->getBodyOrFrameName(body)) {
  set_type(RigidBodyConstraint::WorldGazeTargetConstraintType);
}

void WorldGazeTargetConstraint::eval(const double* t,
                                     KinematicsCache<double>& cache,
                                     VectorXd& c, MatrixXd& dc) const {
  const int num_constraint = getNumConstraint(t);
  int nq = getRobotPointer()->get_num_positions();
  c.resize(num_constraint);
  dc.resize(num_constraint, nq);
  if (isTimeValid(t)) {
    Matrix3Xd body_axis_ends(3, 2);
    body_axis_ends.col(0) = get_gaze_origin();
    body_axis_ends.col(1) = get_gaze_origin() + get_axis();
    nq = getRobotPointer()->get_num_positions();
    auto axis_ends =
        getRobotPointer()->transformPoints(cache, body_axis_ends, body_, 0);
    auto daxis_ends = getRobotPointer()->transformPointsJacobian(
        cache, body_axis_ends, body_, 0, true);

    Vector3d world_axis = axis_ends.col(1) - axis_ends.col(0);
    MatrixXd dworld_axis =
        daxis_ends.block(3, 0, 3, nq) - daxis_ends.block(0, 0, 3, nq);
    Vector3d dir = get_target() - axis_ends.col(0);
    MatrixXd ddir = -daxis_ends.block(0, 0, 3, nq);
    double dir_norm = dir.norm();
    Vector3d dir_normalized = dir / dir_norm;
    MatrixXd ddir_normalized = (MatrixXd::Identity(3, 3) * dir_norm * dir_norm -
                                dir * dir.transpose()) /
                               (std::pow(dir_norm, 3)) * ddir;
    c(0) = dir_normalized.dot(world_axis) - 1.0;
    dc = dir_normalized.transpose() * dworld_axis +
         world_axis.transpose() * ddir_normalized;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void WorldGazeTargetConstraint::name(const double* t,
                                     std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    name_str.push_back(body_name_ + " conic gaze target constraint" + time_str);
  }
}

RelativeGazeTargetConstraint::RelativeGazeTargetConstraint(
    RigidBodyTree* robot, int bodyA_idx, int bodyB_idx,
    const Eigen::Vector3d& axis, const Vector3d& target,
    const Vector3d& gaze_origin, double conethreshold,
    const Eigen::Vector2d& tspan)
    : GazeTargetConstraint(robot, axis, target, gaze_origin, conethreshold,
                           tspan),
      bodyA_idx_(bodyA_idx),
      bodyB_idx_(bodyB_idx),
      bodyA_name_(robot->bodies[bodyA_idx]->get_name()),
      bodyB_name_(robot->bodies[bodyB_idx]->get_name()) {
  set_type(RigidBodyConstraint::RelativeGazeTargetConstraintType);
}

void RelativeGazeTargetConstraint::eval(const double* t,
                                        KinematicsCache<double>& cache,
                                        VectorXd& c, MatrixXd& dc) const {
  if (isTimeValid(t)) {
    int nq = getRobotPointer()->get_num_positions();
    auto target_pos =
        getRobotPointer()->transformPoints(cache, get_target(), bodyB_idx_, 0);
    auto dtarget_pos = getRobotPointer()->transformPointsJacobian(
        cache, get_target(), bodyB_idx_, 0, true);

    auto origin_pos = getRobotPointer()->transformPoints(
        cache, get_gaze_origin(), bodyA_idx_, 0);
    auto dorigin_pos = getRobotPointer()->transformPointsJacobian(
        cache, get_gaze_origin(), bodyA_idx_, 0, true);

    auto axis_pos =
        getRobotPointer()->transformPoints(cache, get_axis(), bodyA_idx_, 0);
    auto daxis_pos = getRobotPointer()->transformPointsJacobian(
        cache, get_axis(), bodyA_idx_, 0, true);

    Vector3d axis_origin = Vector3d::Zero();
    auto axis_origin_pos =
        getRobotPointer()->transformPoints(cache, axis_origin, bodyA_idx_, 0);
    auto daxis_origin_pos = getRobotPointer()->transformPointsJacobian(
        cache, axis_origin, bodyA_idx_, 0, true);

    axis_pos -= axis_origin_pos;
    daxis_pos -= daxis_origin_pos;

    Vector3d origin_to_target = target_pos - origin_pos;
    MatrixXd dorigin_to_target = dtarget_pos - dorigin_pos;
    double origin_to_target_norm = origin_to_target.norm();
    c.resize(1);
    c(0) = origin_to_target.dot(axis_pos) / origin_to_target_norm - 1.0;
    dc.resize(1, nq);
    MatrixXd dcdorigin_to_target =
        (axis_pos.transpose() * origin_to_target_norm -
         axis_pos.dot(origin_to_target) / origin_to_target_norm *
             origin_to_target.transpose()) /
        (origin_to_target_norm * origin_to_target_norm);
    MatrixXd dcdaxis_pos = origin_to_target.transpose() / origin_to_target_norm;
    dc = dcdorigin_to_target * dorigin_to_target + dcdaxis_pos * daxis_pos;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void RelativeGazeTargetConstraint::name(
    const double* t, std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    name_str.push_back(bodyA_name_ + " relative to " + bodyB_name_ +
                       " conic gaze constraint" + time_str);
  }
}

RelativeGazeDirConstraint::RelativeGazeDirConstraint(
    RigidBodyTree* robot, int bodyA_idx, int bodyB_idx, const Vector3d& axis,
    const Vector3d& dir, double conethreshold, const Eigen::Vector2d& tspan)
    : GazeDirConstraint(robot, axis, dir, conethreshold, tspan),
      bodyA_idx_(bodyA_idx),
      bodyB_idx_(bodyB_idx),
      bodyA_name_(robot->bodies[bodyA_idx]->get_name()),
      bodyB_name_(robot->bodies[bodyB_idx]->get_name()) {
  set_type(RigidBodyConstraint::RelativeGazeDirConstraintType);
}

void RelativeGazeDirConstraint::eval(const double* t,
                                     KinematicsCache<double>& cache,
                                     VectorXd& c, MatrixXd& dc) const {
  if (isTimeValid(t)) {
    Matrix3Xd body_axis_ends(3, 2);
    body_axis_ends.block(0, 0, 3, 1) = MatrixXd::Zero(3, 1);
    body_axis_ends.block(0, 1, 3, 1) = get_axis();
    Matrix3Xd body_dir_ends(3, 2);
    body_dir_ends.block(0, 0, 3, 1) = MatrixXd::Zero(3, 1);
    body_dir_ends.block(0, 1, 3, 1) = get_dir();
    int nq = getRobotPointer()->get_num_positions();

    auto axis_pos = getRobotPointer()->transformPoints(cache, body_axis_ends,
                                                       bodyA_idx_, 0);
    auto daxis_pos = getRobotPointer()->transformPointsJacobian(
        cache, body_axis_ends, bodyA_idx_, 0, true);

    auto dir_pos =
        getRobotPointer()->transformPoints(cache, body_dir_ends, bodyB_idx_, 0);
    auto ddir_pos = getRobotPointer()->transformPointsJacobian(
        cache, body_dir_ends, bodyB_idx_, 0, true);

    Vector3d axis_world = axis_pos.col(1) - axis_pos.col(0);
    MatrixXd daxis_world =
        daxis_pos.block(3, 0, 3, nq) - daxis_pos.block(0, 0, 3, nq);
    Vector3d dir_world = dir_pos.col(1) - dir_pos.col(0);
    MatrixXd ddir_world =
        ddir_pos.block(3, 0, 3, nq) - ddir_pos.block(0, 0, 3, nq);
    c.resize(1);
    c(0) = axis_world.dot(dir_world) - 1.0;
    dc = dir_world.transpose() * daxis_world +
         axis_world.transpose() * ddir_world;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void RelativeGazeDirConstraint::name(const double* t,
                                     std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    name_str.push_back(bodyA_name_ + " relative to " + bodyB_name_ +
                       " conic gaze constraint" + time_str);
  }
}

Point2PointDistanceConstraint::Point2PointDistanceConstraint(
    RigidBodyTree* robot, int bodyA, int bodyB, const Matrix3Xd& ptA,
    const Matrix3Xd& ptB, const VectorXd& dist_lb, const VectorXd& dist_ub,
    const Vector2d& tspan)
    : SingleTimeKinematicConstraint(robot, tspan),
      bodyA_(bodyA),
      bodyB_(bodyB),
      ptA_(ptA),
      ptB_(ptB),
      dist_lb_(dist_lb),
      dist_ub_(dist_ub) {
  set_num_constraint(static_cast<int>(ptA.cols()));
  set_type(RigidBodyConstraint::Point2PointDistanceConstraintType);
}

void Point2PointDistanceConstraint::eval(const double* t,
                                         KinematicsCache<double>& cache,
                                         VectorXd& c, MatrixXd& dc) const {
  if (isTimeValid(t)) {
    int num_cnst = getNumConstraint(t);
    MatrixXd posA(3, ptA_.cols());
    MatrixXd dposA(3 * ptA_.cols(),
                   getRobotPointer()->get_num_positions());
    if (bodyA_ != 0) {
      posA = getRobotPointer()->transformPoints(cache, ptA_, bodyA_, 0);
      dposA = getRobotPointer()->transformPointsJacobian(cache, ptA_, bodyA_, 0,
                                                         true);
    } else {
      posA = ptA_.block(0, 0, 3, ptA_.cols());
      dposA = MatrixXd::Zero(3 * ptA_.cols(),
                             getRobotPointer()->get_num_positions());
    }
    MatrixXd posB(3, ptB_.cols());
    MatrixXd dposB(3 * ptB_.cols(),
                   getRobotPointer()->get_num_positions());
    if (bodyB_ != 0) {
      posB = getRobotPointer()->transformPoints(cache, ptB_, bodyB_, 0);
      dposB = getRobotPointer()->transformPointsJacobian(cache, ptB_, bodyB_, 0,
                                                         true);
    } else {
      posB = ptB_.block(0, 0, 3, ptB_.cols());
      dposB = MatrixXd::Zero(3 * ptB_.cols(),
                             getRobotPointer()->get_num_positions());
    }
    MatrixXd d = posA - posB;
    MatrixXd dd = dposA - dposB;
    MatrixXd tmp1 = d.cwiseProduct(d);
    MatrixXd tmp2 = tmp1.colwise().sum();
    c.resize(num_cnst, 1);
    c = tmp2.transpose();
    dc.resize(num_cnst, getRobotPointer()->get_num_positions());
    for (int i = 0; i < num_cnst; i++) {
      dc.row(i) =
          2 * d.col(i).transpose() *
          dd.block(3 * i, 0, 3, getRobotPointer()->get_num_positions());
    }
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void Point2PointDistanceConstraint::name(
    const double* t, std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    int num_cnst = getNumConstraint(t);
    std::string time_str = getTimeString(t);
    for (int i = 0; i < num_cnst; i++) {
      std::string bodyA_name;
      if (bodyA_ != 0) {
        bodyA_name = getRobotPointer()->bodies[bodyA_]->get_name();
      } else {
        bodyA_name = std::string(RigidBodyTree::kWorldName);
      }
      std::string bodyB_name;
      if (bodyB_ != 0) {
        bodyB_name = getRobotPointer()->bodies[bodyB_]->get_name();
      } else {
        bodyB_name = std::string(RigidBodyTree::kWorldName);
      }
      name_str.push_back("Distance from " + bodyA_name + " pt " +
                         std::to_string(i) + " to " + bodyB_name + " pt " +
                         std::to_string(i) + time_str);
    }
  }
}

void Point2PointDistanceConstraint::bounds(const double* t, VectorXd& lb,
                                           VectorXd& ub) const {
  if (isTimeValid(t)) {
    lb = dist_lb_.cwiseProduct(dist_lb_);
    ub = dist_ub_.cwiseProduct(dist_ub_);
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}

Point2LineSegDistConstraint::Point2LineSegDistConstraint(
    RigidBodyTree* robot, int pt_body, const Vector3d& pt, int line_body,
    const Matrix<double, 3, 2>& line_ends, double dist_lb, double dist_ub,
    const Vector2d& tspan)
    : SingleTimeKinematicConstraint(robot, tspan),
      pt_body_(pt_body),
      pt_(pt),
      line_body_(line_body),
      line_ends_(line_ends),
      dist_lb_(dist_lb),
      dist_ub_(dist_ub) {
  set_num_constraint(2);
  set_type(RigidBodyConstraint::Point2LineSegDistConstraintType);
}

void Point2LineSegDistConstraint::eval(const double* t_in,
                                       KinematicsCache<double>& cache,
                                       VectorXd& c, MatrixXd& dc) const {
  if (isTimeValid(t_in)) {
    int nq = getRobotPointer()->get_num_positions();

    auto pt_pos = getRobotPointer()->transformPoints(cache, pt_, pt_body_, 0);
    auto J_pt = getRobotPointer()->transformPointsJacobian(cache, pt_, pt_body_,
                                                           0, true);

    auto line_pos =
        getRobotPointer()->transformPoints(cache, line_ends_, line_body_, 0);
    auto J_line = getRobotPointer()->transformPointsJacobian(
        cache, line_ends_, line_body_, 0, true);

    Vector3d x0 = pt_pos;
    Vector3d x1 = line_pos.col(0);
    Vector3d x2 = line_pos.col(1);
    Vector3d x21 = x2 - x1;
    MatrixXd J21 = J_line.block(3, 0, 3, nq) - J_line.block(0, 0, 3, nq);
    Vector3d x10 = x1 - x0;
    MatrixXd J10 = J_line.block(0, 0, 3, nq) - J_pt;
    double x21_square = x21.squaredNorm();
    double t = -x10.dot(x21) / x21_square;
    MatrixXd dtdx10 = -x21.transpose() / x21_square;
    MatrixXd dtdx21 =
        -(x10.transpose() * x21_square - x10.dot(x21) * 2.0 * x21.transpose()) /
        (x21_square * x21_square);
    MatrixXd dtdq = dtdx10 * J10 + dtdx21 * J21;
    Vector3d h = x0 - (x1 + x21 * t);
    MatrixXd dhdq = J_pt - (J_line.block(0, 0, 3, nq) + J21 * t + x21 * dtdq);
    double d = h.squaredNorm();
    MatrixXd dddq = 2 * h.transpose() * dhdq;
    c.resize(2);
    dc.resize(2, nq);
    c << d, t;
    dc.block(0, 0, 1, nq) = dddq;
    dc.block(1, 0, 1, nq) = dtdq;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void Point2LineSegDistConstraint::bounds(const double* t, VectorXd& lb,
                                         VectorXd& ub) const {
  if (isTimeValid(t)) {
    lb.resize(2);
    ub.resize(2);
    lb << dist_lb_ * dist_lb_, 0.0;
    ub << dist_ub_ * dist_ub_, 1.0;
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}

void Point2LineSegDistConstraint::name(
    const double* t, std::vector<std::string>& name_str) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    name_str.push_back("Distance from " +
                       getRobotPointer()->bodies[pt_body_]->get_name() +
                       " pt to a line on " +
                       getRobotPointer()->bodies[line_body_]->get_name() +
                       time_str);
    name_str.push_back("Fraction of point projection onto line segment " +
                       time_str);
  }
}

WorldFixedPositionConstraint::WorldFixedPositionConstraint(
    RigidBodyTree* robot, int body, const Matrix3Xd& pts, const Vector2d& tspan)
    : MultipleTimeKinematicConstraint(robot, tspan),
      body_(body),
      body_name_(robot->getBodyOrFrameName(body)),
      pts_(pts) {
  if (pts_.rows() != 3) {
    throw std::runtime_error("pts must have 3 rows");
  }
  set_type(RigidBodyConstraint::WorldFixedPositionConstraintType);
}

int WorldFixedPositionConstraint::getNumConstraint(const double* t,
                                                   int n_breaks) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2) {
    return static_cast<int>(pts_.cols());
  } else {
    return 0;
  }
}

void WorldFixedPositionConstraint::eval_valid(const double* valid_t,
                                              int num_valid_t,
                                              const MatrixXd& valid_q,
                                              VectorXd& c,
                                              MatrixXd& dc_valid) const {
  // TODO(tkoolen): don't use raw pointers
  int n_pts = static_cast<int>(pts_.cols());
  int nq = getRobotPointer()->get_num_positions();
  MatrixXd* pos = new MatrixXd[num_valid_t];
  MatrixXd* dpos = new MatrixXd[num_valid_t];

  for (int i = 0; i < num_valid_t; i++) {
    KinematicsCache<double> cache =
        getRobotPointer()->doKinematics(valid_q.col(i));
    pos[i].resize(3, n_pts);
    pos[i] = getRobotPointer()->transformPoints(cache, pts_, body_, 0);
    dpos[i] =
        getRobotPointer()->transformPointsJacobian(cache, pts_, body_, 0, true);
  }
  int* next_idx = new int[num_valid_t];
  int* prev_idx = new int[num_valid_t];
  for (int i = 0; i < num_valid_t; i++) {
    next_idx[i] = (i + 1) % num_valid_t;
    prev_idx[i] = (i + num_valid_t - 1) % num_valid_t;
  }
  c = VectorXd::Zero(n_pts);
  dc_valid = MatrixXd::Zero(n_pts, nq * num_valid_t);
  for (int i = 0; i < num_valid_t; i++) {
    MatrixXd tmp1(3, n_pts);
    tmp1 = pos[i] - pos[next_idx[i]];
    MatrixXd tmp2 = tmp1.cwiseProduct(tmp1);
    VectorXd tmp3 = tmp2.colwise().sum();
    c += tmp3.transpose();
    for (int j = 0; j < n_pts; j++) {
      Vector3d tmp_vec = 4 * pos[i].col(j) - 2 * pos[next_idx[i]].col(j) -
                         2 * pos[prev_idx[i]].col(j);
      dc_valid.block(j, i * nq, 1, nq) =
          tmp_vec.transpose() * dpos[i].block(j * 3, 0, 3, nq);
    }
  }
  delete[] pos;
  delete[] dpos;
  delete[] next_idx;
  delete[] prev_idx;
}

void WorldFixedPositionConstraint::bounds(const double* t, int n_breaks,
                                          VectorXd& lb, VectorXd& ub) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2) {
    int n_pts = static_cast<int>(pts_.cols());
    lb.resize(n_pts, 1);
    ub.resize(n_pts, 1);
    lb = VectorXd::Zero(n_pts);
    ub = VectorXd::Zero(n_pts);
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}

void WorldFixedPositionConstraint::name(
    const double* t, int n_breaks, std::vector<std::string>& name_str) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2) {
    int n_pts = static_cast<int>(pts_.cols());
    for (int i = 0; i < n_pts; i++) {
      name_str.push_back("World fixed position constraint for " + body_name_ +
                         " " + std::to_string(i) + " point");
    }
  }
}

WorldFixedOrientConstraint::WorldFixedOrientConstraint(RigidBodyTree* robot,
                                                       int body,
                                                       const Vector2d& tspan)
    : MultipleTimeKinematicConstraint(robot, tspan),
      body_(body),
      body_name_(robot->getBodyOrFrameName(body)) {
  set_type(RigidBodyConstraint::WorldFixedOrientConstraintType);
}

int WorldFixedOrientConstraint::getNumConstraint(const double* t,
                                                 int n_breaks) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2)
    return 1;
  else
    return 0;
}

void WorldFixedOrientConstraint::eval_valid(const double* valid_t,
                                            int num_valid_t,
                                            const MatrixXd& valid_q,
                                            VectorXd& c,
                                            MatrixXd& dc_valid) const {
  int nq = getRobotPointer()->get_num_positions();
  Vector4d* quat = new Vector4d[num_valid_t];
  MatrixXd* dquat = new MatrixXd[num_valid_t];
  for (int i = 0; i < num_valid_t; i++) {
    KinematicsCache<double> cache =
        getRobotPointer()->doKinematics(valid_q.col(i));
    quat[i] = getRobotPointer()->relativeQuaternion(cache, body_, 0);
    dquat[i].resize(4, nq);
    dquat[i] =
        getRobotPointer()->relativeQuaternionJacobian(cache, body_, 0, true);
  }
  int* next_idx = new int[num_valid_t];
  int* prev_idx = new int[num_valid_t];
  for (int i = 0; i < num_valid_t; i++) {
    next_idx[i] = (i + 1) % num_valid_t;
    prev_idx[i] = (i + num_valid_t - 1) % num_valid_t;
  }
  c(0) = 0.0;
  dc_valid = MatrixXd::Zero(1, nq * num_valid_t);
  for (int i = 0; i < num_valid_t; i++) {
    double tmp1 = quat[i].transpose() * quat[next_idx[i]];
    double tmp2 = quat[i].transpose() * quat[prev_idx[i]];
    c(0) += pow(tmp1, 2);
    dc_valid.block(0, nq * i, 1, nq) =
        (2 * tmp1 * quat[next_idx[i]].transpose() +
         2 * tmp2 * quat[prev_idx[i]].transpose()) *
        dquat[i];
  }
  delete[] quat;
  delete[] dquat;
  delete[] next_idx;
  delete[] prev_idx;
}

void WorldFixedOrientConstraint::bounds(const double* t, int n_breaks,
                                        VectorXd& lb, VectorXd& ub) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2) {
    lb.resize(1);
    ub.resize(1);
    lb(0) = (double)num_valid_t;
    ub(0) = (double)num_valid_t;
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}

void WorldFixedOrientConstraint::name(
    const double* t, int n_breaks, std::vector<std::string>& name_str) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2) {
    name_str.push_back("World fixed orientation constraint for " + body_name_);
  }
}

WorldFixedBodyPoseConstraint::WorldFixedBodyPoseConstraint(
    RigidBodyTree* robot, int body, const Vector2d& tspan)
    : MultipleTimeKinematicConstraint(robot, tspan),
      body_(body),
      body_name_(robot->getBodyOrFrameName(body)) {
  set_type(RigidBodyConstraint::WorldFixedBodyPoseConstraintType);
}

int WorldFixedBodyPoseConstraint::getNumConstraint(const double* t,
                                                   int n_breaks) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2) {
    return 2;
  } else {
    return 0;
  }
}

void WorldFixedBodyPoseConstraint::eval_valid(const double* valid_t,
                                              int num_valid_t,
                                              const MatrixXd& valid_q,
                                              VectorXd& c,
                                              MatrixXd& dc_valid) const {
  // TODO(tkoolen): don't use raw pointers
  int nq = getRobotPointer()->get_num_positions();
  Vector3d* pos = new Vector3d[num_valid_t];
  Vector4d* quat = new Vector4d[num_valid_t];
  MatrixXd* dpos = new MatrixXd[num_valid_t];
  MatrixXd* dquat = new MatrixXd[num_valid_t];
  for (int i = 0; i < num_valid_t; i++) {
    KinematicsCache<double> cache =
        getRobotPointer()->doKinematics(valid_q.col(i));
    Vector3d origin = Vector3d::Zero();
    pos[i] = getRobotPointer()->transformPoints(cache, origin, body_, 0);
    quat[i] = getRobotPointer()->relativeQuaternion(cache, body_, 0);
    dpos[i].resize(3, nq);
    dpos[i] = getRobotPointer()->transformPointsJacobian(cache, origin, body_,
                                                         0, true);
    dquat[i].resize(4, nq);
    dquat[i] =
        getRobotPointer()->relativeQuaternionJacobian(cache, body_, 0, true);
  }
  int* next_idx = new int[num_valid_t];
  int* prev_idx = new int[num_valid_t];
  for (int i = 0; i < num_valid_t; i++) {
    next_idx[i] = (i + 1) % num_valid_t;
    prev_idx[i] = (i + num_valid_t - 1) % num_valid_t;
  }
  c = Vector2d::Zero();
  dc_valid = MatrixXd::Zero(2, nq * num_valid_t);
  for (int i = 0; i < num_valid_t; i++) {
    Vector3d tmp1;
    tmp1 = pos[i] - pos[next_idx[i]];
    c(0) += tmp1.transpose() * tmp1;
    Vector3d tmp_vec = 4 * pos[i] - 2 * pos[next_idx[i]] - 2 * pos[prev_idx[i]];
    dc_valid.block(0, i * nq, 1, nq) = tmp_vec.transpose() * dpos[i];
    double tmp2 = quat[i].transpose() * quat[next_idx[i]];
    double tmp3 = quat[i].transpose() * quat[prev_idx[i]];
    c(1) += pow(tmp2, 2);
    dc_valid.block(1, nq * i, 1, nq) =
        (2 * tmp2 * quat[next_idx[i]].transpose() +
         2 * tmp3 * quat[prev_idx[i]].transpose()) *
        dquat[i];
  }
  delete[] pos;
  delete[] dpos;
  delete[] quat;
  delete[] dquat;
  delete[] next_idx;
  delete[] prev_idx;
}

void WorldFixedBodyPoseConstraint::bounds(const double* t, int n_breaks,
                                          VectorXd& lb, VectorXd& ub) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2) {
    lb.resize(2);
    ub.resize(2);
    lb << 0.0, (double)num_valid_t;
    ub << 0.0, (double)num_valid_t;
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}

void WorldFixedBodyPoseConstraint::name(
    const double* t, int n_breaks, std::vector<std::string>& name_str) const {
  int num_valid_t = numValidTime(t, n_breaks);
  if (num_valid_t >= 2) {
    name_str.push_back("World fixed body pose constraint for " + body_name_ +
                       " position");
    name_str.push_back("World fixed body pose constraint for " + body_name_ +
                       " orientation");
  }
}

AllBodiesClosestDistanceConstraint::AllBodiesClosestDistanceConstraint(
    RigidBodyTree* robot, double lb, double ub,
    const std::vector<int>& active_bodies_idx,
    const std::set<std::string>& active_group_names, const Vector2d& tspan)
    : SingleTimeKinematicConstraint(robot, tspan),
      ub_(ub),
      lb_(lb),
      active_bodies_idx_(active_bodies_idx),
      active_group_names_(active_group_names) {
  set_type(RigidBodyConstraint::AllBodiesClosestDistanceConstraintType);
  updateRobot(robot);
}

void AllBodiesClosestDistanceConstraint::updateRobot(RigidBodyTree* robot) {
  set_robot(robot);
  double t = 0;
  VectorXd c;
  MatrixXd dc;

  // FIXME: hack to determine num_constraint
  VectorXd q = getRobotPointer()->getZeroConfiguration();
  KinematicsCache<double> cache = getRobotPointer()->doKinematics(q);
  eval(&t, cache, c, dc);

  set_num_constraint(static_cast<int>(c.size()));
}

void AllBodiesClosestDistanceConstraint::eval(const double* t,
                                              KinematicsCache<double>& cache,
                                              VectorXd& c, MatrixXd& dc) const {
  if (isTimeValid(t)) {
    Matrix3Xd xA, xB, normal;
    std::vector<int> idxA;
    std::vector<int> idxB;

    if (active_bodies_idx_.size() > 0) {
      if (active_group_names_.size() > 0) {
        getRobotPointer()->collisionDetect(cache, c, normal, xA, xB, idxA, idxB,
                                           active_bodies_idx_,
                                           active_group_names_);
      } else {
        getRobotPointer()->collisionDetect(cache, c, normal, xA, xB, idxA, idxB,
                                           active_bodies_idx_);
      }
    } else {
      if (active_group_names_.size() > 0) {
        getRobotPointer()->collisionDetect(cache, c, normal, xA, xB, idxA, idxB,
                                           active_group_names_);
      } else {
        getRobotPointer()->collisionDetect(cache, c, normal, xA, xB, idxA,
                                           idxB);
      }
    }
    int num_pts = static_cast<int>(xA.cols());
    dc = MatrixXd::Zero(num_pts, getRobotPointer()->get_num_positions());
    MatrixXd JA = MatrixXd::Zero(3, getRobotPointer()->get_num_positions());
    MatrixXd JB = MatrixXd::Zero(3, getRobotPointer()->get_num_positions());
    for (int i = 0; i < num_pts; ++i) {
      JA = getRobotPointer()->transformPointsJacobian(cache, xA.col(i),
                                                      idxA.at(i), 0, true);
      JB = getRobotPointer()->transformPointsJacobian(cache, xB.col(i),
                                                      idxB.at(i), 0, true);
      dc.row(i) = normal.col(i).transpose() * (JA - JB);
    }
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void AllBodiesClosestDistanceConstraint::bounds(const double* t, VectorXd& lb,
                                                VectorXd& ub) const {
  const int num_constraint = getNumConstraint(t);
  lb.resize(num_constraint);
  ub.resize(num_constraint);
  if (isTimeValid(t)) {
    lb = VectorXd::Constant(num_constraint, lb_);
    ub = VectorXd::Constant(num_constraint, ub_);
  }
}

void AllBodiesClosestDistanceConstraint::name(
    const double* t, std::vector<std::string>& name) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    std::string cnst_name = "All-to-all closest distance constraint" + time_str;
    const int num_constraint = getNumConstraint(t);
    for (int i = 0; i < num_constraint; i++) {
      name.push_back(cnst_name);
    }
  } else {
    name.push_back("");
  }
}

MinDistanceConstraint::MinDistanceConstraint(
    RigidBodyTree* robot, double min_distance,
    const std::vector<int>& active_bodies_idx,
    const std::set<std::string>& active_group_names, const Vector2d& tspan)
    : SingleTimeKinematicConstraint(robot, tspan),
      min_distance_(min_distance),
      active_bodies_idx_(active_bodies_idx),
      active_group_names_(active_group_names) {
  set_num_constraint(1);
  set_type(RigidBodyConstraint::MinDistanceConstraintType);
}

void MinDistanceConstraint::eval(const double* t,
                                 KinematicsCache<double>& cache, VectorXd& c,
                                 MatrixXd& dc) const {
  // DEBUG
  // std::cout << "MinDistanceConstraint::eval: START" << std::endl;
  // END_DEBUG
  if (isTimeValid(t)) {
    VectorXd dist, scaled_dist, pairwise_costs;
    Matrix3Xd xA, xB, normal;
    MatrixXd ddist_dq, dscaled_dist_ddist, dpairwise_costs_dscaled_dist;
    std::vector<int> idxA;
    std::vector<int> idxB;

    if (active_bodies_idx_.size() > 0) {
      if (active_group_names_.size() > 0) {
        getRobotPointer()->collisionDetect(cache, dist, normal, xA, xB, idxA,
                                           idxB, active_bodies_idx_,
                                           active_group_names_);
      } else {
        getRobotPointer()->collisionDetect(cache, dist, normal, xA, xB, idxA,
                                           idxB, active_bodies_idx_);
      }
    } else {
      if (active_group_names_.size() > 0) {
        getRobotPointer()->collisionDetect(cache, dist, normal, xA, xB, idxA,
                                           idxB, active_group_names_);
      } else {
        getRobotPointer()->collisionDetect(cache, dist, normal, xA, xB, idxA,
                                           idxB);
      }
    }

    int num_pts = static_cast<int>(xA.cols());
    ddist_dq =
        MatrixXd::Zero(num_pts, getRobotPointer()->get_num_positions());

    // Compute Jacobian of closest distance vector
    scaleDistance(dist, scaled_dist, dscaled_dist_ddist);
    penalty(scaled_dist, pairwise_costs, dpairwise_costs_dscaled_dist);

    std::vector<std::vector<int>> orig_idx_of_pt_on_bodyA(
        getRobotPointer()->bodies.size());
    std::vector<std::vector<int>> orig_idx_of_pt_on_bodyB(
        getRobotPointer()->bodies.size());
    for (int k = 0; k < num_pts; ++k) {
      if (pairwise_costs(k) > 0) {
        orig_idx_of_pt_on_bodyA.at(idxA.at(k)).push_back(k);
        orig_idx_of_pt_on_bodyB.at(idxB.at(k)).push_back(k);
      }
    }
    for (int k = 0; k < static_cast<int>(getRobotPointer()->bodies.size());
         ++k) {
      int l = 0;
      int numA = static_cast<int>(orig_idx_of_pt_on_bodyA.at(k).size());
      int numB = static_cast<int>(orig_idx_of_pt_on_bodyB.at(k).size());
      if (numA + numB == 0) {
        continue;
      }
      Matrix3Xd x_k(3, numA + numB);
      for (; l < numA; ++l) {
        x_k.col(l) = xA.col(orig_idx_of_pt_on_bodyA.at(k).at(l));
      }
      for (; l < numA + numB; ++l) {
        x_k.col(l) = xB.col(orig_idx_of_pt_on_bodyB.at(k).at(l - numA));
      }

      auto J_k =
          getRobotPointer()->transformPointsJacobian(cache, x_k, k, 0, true);

      l = 0;
      for (; l < numA; ++l) {
        ddist_dq.row(orig_idx_of_pt_on_bodyA.at(k).at(l)) +=
            normal.col(orig_idx_of_pt_on_bodyA.at(k).at(l)).transpose() *
            J_k.block(3 * l, 0, 3, getRobotPointer()->get_num_positions());
      }
      for (; l < numA + numB; ++l) {
        ddist_dq.row(orig_idx_of_pt_on_bodyB.at(k).at(l - numA)) +=
            -normal.col(orig_idx_of_pt_on_bodyB.at(k).at(l - numA))
                 .transpose() *
            J_k.block(3 * l, 0, 3, getRobotPointer()->get_num_positions());
      }
    }
    MatrixXd dcost_dscaled_dist(dpairwise_costs_dscaled_dist.colwise().sum());
    c.resize(1);
    c(0) = pairwise_costs.sum();
    dc = dcost_dscaled_dist * dscaled_dist_ddist * ddist_dq;
  } else {
    c.resize(0);
    dc.resize(0, 0);
  }
}

void MinDistanceConstraint::scaleDistance(
    const Eigen::VectorXd& dist, Eigen::VectorXd& scaled_dist,
    Eigen::MatrixXd& dscaled_dist_ddist) const {
  int nd = static_cast<int>(dist.size());
  double recip_min_dist = 1 / min_distance_;
  scaled_dist = recip_min_dist * dist - VectorXd::Ones(nd, 1);
  dscaled_dist_ddist = recip_min_dist * MatrixXd::Identity(nd, nd);
}

void MinDistanceConstraint::penalty(const Eigen::VectorXd& dist,
                                    Eigen::VectorXd& cost,
                                    Eigen::MatrixXd& dcost_ddist) const {
  int nd = static_cast<int>(dist.size());
  cost = VectorXd::Zero(nd, 1);
  dcost_ddist = MatrixXd::Zero(nd, nd);
  for (int i = 0; i < nd; ++i) {
    if (dist(i) < 0) {
      double exp_recip_dist = exp(1 / dist(i));
      cost(i) = -dist(i) * exp_recip_dist;
      dcost_ddist(i, i) = exp_recip_dist * (1 / dist(i) - 1);
    }
  }
}

void MinDistanceConstraint::bounds(const double* t, VectorXd& lb,
                                   VectorXd& ub) const {
  const int num_constraint = getNumConstraint(t);
  lb.resize(num_constraint);
  ub.resize(num_constraint);
  if (isTimeValid(t)) {
    lb = VectorXd::Zero(num_constraint);
    ub = VectorXd::Zero(num_constraint);
  }
}

void MinDistanceConstraint::name(const double* t,
                                 std::vector<std::string>& name) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    std::string cnst_name("Minimum distance constraint" + time_str);
    name.push_back(cnst_name);
  } else {
    name.push_back("");
  }
}

WorldPositionInFrameConstraint::WorldPositionInFrameConstraint(
    RigidBodyTree* robot, int body, const Eigen::Matrix3Xd& pts,
    const Eigen::Matrix4d& T_frame_to_world, const Eigen::MatrixXd& lb,
    const Eigen::MatrixXd& ub, const Eigen::Vector2d& tspan)
    : WorldPositionConstraint(robot, body, pts, lb, ub, tspan),
      T_world_to_frame_(T_frame_to_world.inverse()) {
  set_type(RigidBodyConstraint::WorldPositionInFrameConstraintType);
}

void WorldPositionInFrameConstraint::evalPositions(
    KinematicsCache<double>& cache, Matrix3Xd& pos, MatrixXd& J) const {
  WorldPositionConstraint::evalPositions(cache, pos, J);
  pos = (T_world_to_frame_ * pos.colwise().homogeneous()).topRows(3);
  auto J_reshaped = Map<MatrixXd>(J.data(), 3, get_n_pts() * J.cols());
  J_reshaped = T_world_to_frame_.topLeftCorner<3, 3>() * J_reshaped;
}

void WorldPositionInFrameConstraint::evalNames(
    const double* t, std::vector<std::string>& cnst_names) const {
  std::string time_str = getTimeString(t);
  for (int i = 0; i < get_n_pts(); i++) {
    cnst_names.push_back(get_body_name() + " pts(:," + std::to_string(i + 1) +
                         ") x in frame " + time_str);
    cnst_names.push_back(get_body_name() + " pts(:," + std::to_string(i + 1) +
                         ") y in frame " + time_str);
    cnst_names.push_back(get_body_name() + " pts(:," + std::to_string(i + 1) +
                         ") z in frame " + time_str);
  }
}

WorldPositionInFrameConstraint::~WorldPositionInFrameConstraint() {}

PostureChangeConstraint::PostureChangeConstraint(RigidBodyTree* robot,
                                                 const VectorXi& joint_ind,
                                                 const VectorXd& lb_change,
                                                 const VectorXd& ub_change,
                                                 const Vector2d& tspan)
    : MultipleTimeLinearPostureConstraint(robot, tspan) {
  setJointChangeBounds(joint_ind, lb_change, ub_change);
  set_type(RigidBodyConstraint::PostureChangeConstraintType);
}

void PostureChangeConstraint::setJointChangeBounds(const VectorXi& joint_ind,
                                                   const VectorXd& lb_change,
                                                   const VectorXd& ub_change) {
  joint_ind_ = joint_ind;
  lb_change_ = VectorXd(joint_ind.size());
  ub_change_ = VectorXd(joint_ind.size());
  for (int i = 0; i < joint_ind.size(); i++) {
    double lb_change_min =
        getRobotPointer()->joint_limit_min[joint_ind(i)] -
        getRobotPointer()->joint_limit_max[joint_ind(i)];
    double ub_change_max =
        getRobotPointer()->joint_limit_max[joint_ind(i)] -
        getRobotPointer()->joint_limit_min[joint_ind(i)];
    lb_change_(i) =
        (lb_change_min < lb_change(i) ? lb_change(i) : lb_change_min);
    ub_change_(i) =
        (ub_change_max > ub_change(i) ? ub_change(i) : ub_change_max);
  }
}

int PostureChangeConstraint::getNumConstraint(const double* t,
                                              int n_breaks) const {
  std::vector<bool> valid_flag = isTimeValid(t, n_breaks);
  int num_valid_t = numValidTime(valid_flag);
  if (num_valid_t >= 2)
    return (num_valid_t - 1) * static_cast<int>(joint_ind_.size());
  else
    return 0;
}

void PostureChangeConstraint::feval(const double* t, int n_breaks,
                                    const MatrixXd& q, VectorXd& c) const {
  std::vector<bool> valid_flag = isTimeValid(t, n_breaks);
  int num_valid_t = numValidTime(valid_flag);
  if (num_valid_t >= 2) {
    VectorXi valid_t_ind;
    validTimeInd(valid_flag, valid_t_ind);
    int nc = getNumConstraint(t, n_breaks);
    c.resize(nc);
    for (int i = 1; i < num_valid_t; i++) {
      for (int j = 0; j < joint_ind_.size(); j++) {
        c((i - 1) * joint_ind_.size() + j) =
            q(joint_ind_(j), valid_t_ind(i)) - q(joint_ind_(j), valid_t_ind(0));
      }
    }
  } else {
    c.resize(0);
  }
}

void PostureChangeConstraint::geval(const double* t, int n_breaks,
                                    VectorXi& iAfun, VectorXi& jAvar,
                                    VectorXd& A) const {
  std::vector<bool> valid_flag = isTimeValid(t, n_breaks);
  int num_valid_t = numValidTime(valid_flag);
  if (num_valid_t >= 2) {
    int num_joints = static_cast<int>(joint_ind_.size());
    int nc = getNumConstraint(t, n_breaks);
    int nq = getRobotPointer()->get_num_positions();
    iAfun.resize(nc * 2);
    jAvar.resize(nc * 2);
    A.resize(nc * 2);
    VectorXi valid_t_ind;
    validTimeInd(valid_flag, valid_t_ind);
    for (int i = 0; i < num_valid_t - 1; i++) {
      for (int j = 0; j < num_joints; j++) {
        iAfun(i * num_joints + j) = i * num_joints + j;
        jAvar(i * num_joints + j) = valid_t_ind(0) * nq + joint_ind_[j];
        A(i * num_joints + j) = -1;
      }
    }
    for (int i = 1; i < num_valid_t; i++) {
      for (int j = 0; j < num_joints; j++) {
        int ind = (num_valid_t - 1) * num_joints + (i - 1) * num_joints + j;
        iAfun(ind) = (i - 1) * num_joints + j;
        jAvar(ind) = nq * valid_t_ind(i) + joint_ind_[j];
        A(ind) = 1.0;
      }
    }
  } else {
    iAfun.resize(0);
    jAvar.resize(0);
    A.resize(0);
  }
}

void PostureChangeConstraint::name(const double* t, int n_breaks,
                                   std::vector<std::string>& name_str) const {
  std::vector<bool> valid_flag = isTimeValid(t, n_breaks);
  int num_valid_t = numValidTime(valid_flag);
  VectorXi valid_t_ind;
  validTimeInd(valid_flag, valid_t_ind);
  if (num_valid_t >= 2) {
    for (int i = 1; i < num_valid_t; i++) {
      for (int j = 0; j < joint_ind_.size(); j++) {
        name_str.push_back("Posture change for joint " +
                           std::to_string(joint_ind_[j] + 1) + " at time " +
                           std::to_string(t[valid_t_ind(i)]));
      }
    }
  }
}

void PostureChangeConstraint::bounds(const double* t, int n_breaks,
                                     VectorXd& lb, VectorXd& ub) const {
  std::vector<bool> valid_flag = isTimeValid(t, n_breaks);
  int num_valid_t = numValidTime(valid_flag);
  if (num_valid_t >= 2) {
    int nc = getNumConstraint(t, n_breaks);
    lb.resize(nc);
    ub.resize(nc);
    int num_joints = static_cast<int>(joint_ind_.size());
    for (int i = 0; i < num_valid_t - 1; i++) {
      lb.block(i * num_joints, 0, num_joints, 1) = lb_change_;
      ub.block(i * num_joints, 0, num_joints, 1) = ub_change_;
    }
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}

GravityCompensationTorqueConstraint::GravityCompensationTorqueConstraint(
    RigidBodyTree* robot, const VectorXi& joint_indices, const VectorXd& lb,
    const VectorXd& ub, const Vector2d& tspan)
    : SingleTimeKinematicConstraint(robot, tspan),
      joint_indices_(joint_indices),
      lb_(lb),
      ub_(ub) {
  set_num_constraint(static_cast<int>(joint_indices.size()));
  set_type(RigidBodyConstraint::GravityCompensationTorqueConstraintType);
}

void GravityCompensationTorqueConstraint::eval(const double* t,
                                               KinematicsCache<double>& cache,
                                               VectorXd& c,
                                               MatrixXd& dc) const {
  // FIXME: very inefficient:
  typedef AutoDiffScalar<VectorXd> Scalar;
  auto q = cache.getQ().cast<Scalar>().eval();
  gradientMatrixToAutoDiff(
      MatrixXd::Identity(getRobotPointer()->get_num_positions(),
                         getRobotPointer()->get_num_positions()),
      q);
  KinematicsCache<Scalar> cache_with_gradients =
      getRobotPointer()->doKinematics(q);
  const RigidBodyTree::BodyToWrenchMap<Scalar> no_external_wrenches;
  auto G_autodiff = getRobotPointer()->dynamicsBiasTerm(
      cache_with_gradients, no_external_wrenches, false);
  auto G = autoDiffToValueMatrix(G_autodiff);
  auto dG = autoDiffToGradientMatrix(G_autodiff);

  const int num_constraint = getNumConstraint(t);
  c.resize(num_constraint);
  dc.resize(num_constraint, getRobotPointer()->get_num_positions());

  for (int i = 0; i < num_constraint; ++i) {
    c(i) = G(joint_indices_(i));
    dc.row(i) = dG.block(joint_indices_(i), 0, 1,
                         getRobotPointer()->get_num_positions());
  }
}

void GravityCompensationTorqueConstraint::name(
    const double* t, std::vector<std::string>& name) const {
  if (isTimeValid(t)) {
    std::string time_str = getTimeString(t);
    std::string cnst_name = "Gravity compensation torque constraint" + time_str;
    const int num_constraint = getNumConstraint(t);
    for (int i = 0; i < num_constraint; i++) {
      name.push_back(cnst_name);
    }
  } else {
    name.push_back("");
  }
}

void GravityCompensationTorqueConstraint::bounds(const double* t, VectorXd& lb,
                                                 VectorXd& ub) const {
  if (isTimeValid(t)) {
    lb = lb_;
    ub = ub_;
  } else {
    lb.resize(0);
    ub.resize(0);
  }
}
