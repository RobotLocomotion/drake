#pragma once

// TODO(#2274) NOTE This file has so many cpplint errors that we have
// whitelisted it in its entirety.  When the file is next rewritten or updates,
// we should re-enable cpplint accordingly.

#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/KinematicsCache.h"

class RigidBodyTree;

namespace DrakeRigidBodyConstraint {
extern DRAKE_EXPORT Eigen::Vector2d default_tspan;
}

/**
 * @class RigidBodyConstraint       The abstract base class. All constraints
 * used in the inverse kinematics problem are inherited from
 * RigidBodyConstraint. There are 6 main categories of the RigidBodyConstraint,
 * each category has its own interface
 */
class DRAKE_EXPORT RigidBodyConstraint {
 public:
  /* In each category, constraint classes share the same function interface, this
   * value needs to be in consistent with that in MATLAB*/
  static const int SingleTimeKinematicConstraintCategory = -1;
  static const int MultipleTimeKinematicConstraintCategory = -2;
  static const int QuasiStaticConstraintCategory = -3;
  static const int PostureConstraintCategory = -4;
  static const int MultipleTimeLinearPostureConstraintCategory = -5;
  static const int SingleTimeLinearPostureConstraintCategory = -6;
  /* Each non-abstrac RigidBodyConstraint class has a unique type. Make sure
   * this value stays in consistent with the value in MATLAB*/
  static const int QuasiStaticConstraintType = 1;
  static const int PostureConstraintType = 2;
  static const int SingleTimeLinearPostureConstraintType = 3;
  static const int AllBodiesClosestDistanceConstraintType = 4;
  static const int WorldEulerConstraintType = 5;
  static const int WorldGazeDirConstraintType = 6;
  static const int WorldGazeOrientConstraintType = 7;
  static const int WorldGazeTargetConstraintType = 8;
  static const int RelativeGazeTargetConstraintType = 9;
  static const int WorldCoMConstraintType = 10;
  static const int WorldPositionConstraintType = 11;
  static const int WorldPositionInFrameConstraintType = 12;
  static const int WorldQuatConstraintType = 13;
  static const int Point2PointDistanceConstraintType = 14;
  static const int Point2LineSegDistConstraintType = 15;
  static const int WorldFixedPositionConstraintType = 16;
  static const int WorldFixedOrientConstraintType = 17;
  static const int WorldFixedBodyPoseConstraintType = 18;
  static const int PostureChangeConstraintType = 19;
  static const int RelativePositionConstraintType = 20;
  static const int RelativeQuatConstraintType = 24;
  static const int RelativeGazeDirConstraintType = 25;
  static const int MinDistanceConstraintType = 26;
  static const int GravityCompensationTorqueConstraintType = 27;

  RigidBodyConstraint(
      int category, RigidBodyTree* robot,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  int getType() const { return type_; }
  int getCategory() const { return category_; }
  RigidBodyTree* getRobotPointer() const { return robot_; }
  virtual ~RigidBodyConstraint(void) = 0;

 protected:
  std::string getTimeString(const double* t) const;
  void set_type(int type) { type_ = type; }
  void set_robot(RigidBodyTree* robot) { robot_ = robot; }
  const double* tspan() const { return tspan_; }

 private:
  int category_{};
  int type_{};
  RigidBodyTree* robot_{};
  double tspan_[2];
};

/**
 * @class QuasiStaticConstraint       -- Constrain the Center of Mass (CoM) is
 * within the support polygon. The support polygon is a shrunk area of the
 * contact polygon
 * @param robot
 * @param tspan           -- The time span of this constraint being active
 * @param model_instance_id_set   -- The set of the robots in the RigidBodyTree
 * for which the CoM is computed
 * @param shrinkFactor    -- The factor to shrink the contact polygon. The
 * shrunk area is the support polygon.
 * @param active          -- Whether the constraint is on/off. If active =
 * false, even the time t is within tspan, the constraint is still inactive
 * @param num_bodies      -- The total number of ground contact bodies/frames
 * @param num_pts         -- The total number of ground contact points
 * @param bodies          -- The index of ground contact bodies/frames
 * @param num_body_pts    -- The number of contact points on each contact
 * body/frame
 * @param body_pts        -- The contact points on each contact body/frame
 *
 * Function:
 *  @function eval     --evaluate the constraint
 *    @param t       --the time to evaluate the constraint
 *    @param weights --the weight associate with each ground contact point
 *    @param c       -- c = CoM-weights'*support_vertex
 *    @param dc      -- dc = [dcdq dcdweiths]
 *  @function addContact  -- add contact body and points
 *    @param num_new_bodies      -- number of new contact bodies
 *    @param body                -- the index of new contact bodies/frames
 *    @param body_pts            -- body_pts[i] are the contact points on
 * body[i]
 */

class DRAKE_EXPORT QuasiStaticConstraint
    : public RigidBodyConstraint {
 public:
  QuasiStaticConstraint(
      RigidBodyTree* robot,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan,
      const std::set<int>& model_instance_id_set =
          QuasiStaticConstraint::defaultRobotNumSet);
  virtual ~QuasiStaticConstraint(void);
  bool isTimeValid(const double* t) const;
  int getNumConstraint(const double* t) const;
  void eval(const double* t, KinematicsCache<double>& cache,
            const double* weights, Eigen::VectorXd& c,
            Eigen::MatrixXd& dc) const;
  void bounds(const double* t, Eigen::VectorXd& lb, Eigen::VectorXd& ub) const;
  void name(const double* t, std::vector<std::string>& name_str) const;
  bool isActive() const { return active_; }
  int getNumWeights() const { return num_pts_; }
  void addContact(int num_new_bodies, const int* body,
                  const Eigen::Matrix3Xd* body_pts);

  void addContact(std::vector<int> body, const Eigen::Matrix3Xd& body_pts) {
    addContact(body.size(), body.data(), &body_pts);
  }

  void setShrinkFactor(double factor);
  void setActive(bool flag) { active_ = flag; }
  void updateRobot(RigidBodyTree* robot);
  void updateRobotnum(std::set<int>& model_instance_id_set);

 private:
  static const std::set<int> defaultRobotNumSet;
  std::set<int> m_model_instance_id_set_;
  double shrink_factor_{};
  bool active_{};
  int num_bodies_{};
  int num_pts_{};
  std::vector<int> bodies_;
  std::vector<int> num_body_pts_;
  std::vector<Eigen::Matrix3Xd> body_pts_;
};

/*
 * @class PostureConstraint   constrain the joint limits
 * @param tspan          -- The time span of the constraint being valid
 * @param lb             -- The lower bound of the joints
 * @param ub             -- The upper bound of the joints
 *
 * @function setJointLimits   set the limit of some joints
 *   @param num_idx    The number of joints whose limits are going to be set
 *   @param joint_idx  joint_idx[i] is the index of the i'th joint whose limits
 * are going to be set
 *   @param lb         lb[i] is the lower bound of the joint joint_idx[i]
 *   @param ub         ub[i] is the upper bound of the joint joint_idx[i]
 */
class DRAKE_EXPORT PostureConstraint
    : public RigidBodyConstraint {
 public:
  PostureConstraint(
      RigidBodyTree* model,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~PostureConstraint(void) {}
  bool isTimeValid(const double* t) const;
  void setJointLimits(int num_idx, const int* joint_idx,
                      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);
  void setJointLimits(const Eigen::VectorXi& joint_idx,
                      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);
  void bounds(const double* t, Eigen::VectorXd& joint_min,
              Eigen::VectorXd& joint_max) const;

 private:
  Eigen::VectorXd joint_limit_min0_;
  Eigen::VectorXd joint_limit_max0_;
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
};

/*
 * @class MultipleTimeLinearPostureConstraint constrain the posture such that
 * lb(t(1), t(2),..., t(n)) <=
 * A_mat(t(1), t(2), t(n))*[q(t(1));q(t(2));...;q(t(n))] <=
 * ub(t(1), t(2),..., t(n))
 * where A_mat is a sparse matrix that only depends on t(1), t(2),..., t(n)
 *
 * @function eval return the value and gradient of the constraint
 *   @param t      array of time
 *   @param n_breaks   the length of array t
 *   @param q     q.col(i) is the posture at t[i]
 *   @param c    the value of the constraint
 *   @param dc   the gradient of the constraint w.r.t. q
 *
 * @function feval returns the value of the constraint
 *
 * @function geval returns the gradient of the constraint, written in the sprase
 * matrix form
 *   @return iAfun    The row index of the non-zero entries in the gradient
 * matrix
 *   @return jAvar    The column index of the non-zero entries in the gradient
 * matrix
 *   @return A        The value of the non-zero entries in the gradient matrix
 */
class DRAKE_EXPORT MultipleTimeLinearPostureConstraint
    : public RigidBodyConstraint {
 public:
  MultipleTimeLinearPostureConstraint(
      RigidBodyTree* model,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~MultipleTimeLinearPostureConstraint() {}
  std::vector<bool> isTimeValid(const double* t, int n_breaks) const;
  void eval(const double* t, int n_breaks, const Eigen::MatrixXd& q,
            Eigen::VectorXd& c, Eigen::SparseMatrix<double>& dc) const;
  virtual int getNumConstraint(const double* t, int n_breaks) const = 0;
  virtual void feval(const double* t, int n_breaks, const Eigen::MatrixXd& q,
                     Eigen::VectorXd& c) const = 0;
  virtual void geval(const double* t, int n_breaks, Eigen::VectorXi& iAfun,
                     Eigen::VectorXi& jAvar, Eigen::VectorXd& A) const = 0;
  virtual void name(const double* t, int n_breaks,
                    std::vector<std::string>& name_str) const = 0;
  virtual void bounds(const double* t, int n_breaks, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const = 0;

 protected:
  int numValidTime(const std::vector<bool>& valid_flag) const;
  void validTimeInd(const std::vector<bool>& valid_flag,
                    Eigen::VectorXi& valid_t_ind) const;
};

/*
 * @class SingleTimeLinearPostureConstraint constrain the posture satisfies lb<=
 * A_mat*q <=ub at any time, where A_mat is a sparse matrix
 *
 * @function SingleTimeLinearPostureConstraint
 *   @param robot
 *   @param iAfun    The row indices of non zero entries
 *   @param jAvar    The column indices of non zero entries
 *   @param A        The values of non zero entries
 *   @param lb       The lower bound of the constraint, a column vector.
 *   @param ub       The upper bound of the constraint, a column vector.
 *   @param tspan    The time span [tspan[0] tspan[1]] is the time span of the
 * constraint being active
 * @function eval return the value and gradient of the constraint
 *   @param t      array of time
 *   @param n_breaks   the length of array t
 *   @param q     q.col(i) is the posture at t[i]
 *   @param c    the value of the constraint
 *   @param dc   the gradient of the constraint w.r.t. q
 *
 * @function feval returns the value of the constraint
 *
 * @function geval returns the gradient of the constraint, written in the sprase
 * matrix form
 *   @return iAfun    The row index of the non-zero entries in the gradient
 * matrix
 *   @return jAvar    The column index of the non-zero entries in the gradient
 * matrix
 *   @return A        The value of the non-zero entries in the gradient matrix
 */
class DRAKE_EXPORT SingleTimeLinearPostureConstraint
    : public RigidBodyConstraint {
 public:
  SingleTimeLinearPostureConstraint(
      RigidBodyTree* robot, const Eigen::VectorXi& iAfun,
      const Eigen::VectorXi& jAvar, const Eigen::VectorXd& A,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~SingleTimeLinearPostureConstraint(void) {}
  bool isTimeValid(const double* t) const;
  int getNumConstraint(const double* t) const;
  void bounds(const double* t, Eigen::VectorXd& lb, Eigen::VectorXd& ub) const;
  void feval(const double* t, const Eigen::VectorXd& q,
             Eigen::VectorXd& c) const;
  void geval(const double* t, Eigen::VectorXi& iAfun, Eigen::VectorXi& jAvar,
             Eigen::VectorXd& A) const;
  void eval(const double* t, const Eigen::VectorXd& q, Eigen::VectorXd& c,
            Eigen::SparseMatrix<double>& dc) const;
  void name(const double* t, std::vector<std::string>& name_str) const;

 private:
  Eigen::VectorXi iAfun_;
  Eigen::VectorXi jAvar_;
  Eigen::VectorXd A_;
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
  int num_constraint_{};
  Eigen::SparseMatrix<double> A_mat_;
};

/*
 * class SingleTimeKinematicConstraint   An abstract class that constrain the
 * kinematics of the robot at individual time. Need to call doKinematics first
 * for the robot and then evaulate this constraint.
 */
class DRAKE_EXPORT SingleTimeKinematicConstraint
    : public RigidBodyConstraint {
 public:
  SingleTimeKinematicConstraint(
      RigidBodyTree* model,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~SingleTimeKinematicConstraint() {}
  bool isTimeValid(const double* t) const;
  int getNumConstraint(const double* t) const;
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const = 0;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const = 0;
  virtual void name(const double* t,
                    std::vector<std::string>& name_str) const = 0;
  virtual void updateRobot(RigidBodyTree* robot);

 protected:
  void set_num_constraint(int num_constraint) {
    num_constraint_ = num_constraint;
  }

 private:
  int num_constraint_{};
};

class DRAKE_EXPORT MultipleTimeKinematicConstraint
    : public RigidBodyConstraint {
 public:
  MultipleTimeKinematicConstraint(
      RigidBodyTree* model,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~MultipleTimeKinematicConstraint() {}
  std::vector<bool> isTimeValid(const double* t, int n_breaks) const;
  virtual int getNumConstraint(const double* t, int n_breaks) const = 0;
  void eval(const double* t, int n_breaks, const Eigen::MatrixXd& q,
            Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void eval_valid(const double* valid_t, int num_valid_t,
                          const Eigen::MatrixXd& valid_q, Eigen::VectorXd& c,
                          Eigen::MatrixXd& dc_valid) const = 0;
  virtual void bounds(const double* t, int n_breaks, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const = 0;
  virtual void name(const double* t, int n_breaks,
                    std::vector<std::string>& name_str) const = 0;
  virtual void updateRobot(RigidBodyTree* robot);

 protected:
  int numValidTime(const double* t, int n_breaks) const;
};

class DRAKE_EXPORT PositionConstraint
    : public SingleTimeKinematicConstraint {
 public:
  PositionConstraint(
      RigidBodyTree* model, const Eigen::Matrix3Xd& pts, Eigen::MatrixXd lb,
      Eigen::MatrixXd ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~PositionConstraint(void) {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;
  virtual void name(const double* t, std::vector<std::string>& name_str) const;

 protected:
  virtual void evalPositions(KinematicsCache<double>& cache,
                             Eigen::Matrix3Xd& pos,
                             Eigen::MatrixXd& J) const = 0;
  virtual void evalNames(const double* t,
                         std::vector<std::string>& cnst_names) const = 0;
  const Eigen::Matrix3Xd& get_pts() const { return pts_; }
  int get_n_pts() const { return n_pts_; }

 private:
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
  std::vector<bool> null_constraint_rows_;
  Eigen::Matrix3Xd pts_;
  int n_pts_{};
};

class DRAKE_EXPORT WorldPositionConstraint
    : public PositionConstraint {
 public:
  WorldPositionConstraint(
      RigidBodyTree* model, int body, const Eigen::Matrix3Xd& pts,
      Eigen::MatrixXd lb, Eigen::MatrixXd ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldPositionConstraint();

 protected:
  virtual void evalPositions(KinematicsCache<double>& cache,
                             Eigen::Matrix3Xd& pos, Eigen::MatrixXd& J) const;
  virtual void evalNames(const double* t,
                         std::vector<std::string>& cnst_names) const;
  int get_body() const { return body_; }
  const std::string& get_body_name() const { return body_name_; }

 private:
  int body_{};
  std::string body_name_;
};

class DRAKE_EXPORT WorldCoMConstraint
    : public PositionConstraint {
 public:
  WorldCoMConstraint(
      RigidBodyTree* model, Eigen::Vector3d lb, Eigen::Vector3d ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan,
      const std::set<int>& model_instance_id =
          WorldCoMConstraint::defaultRobotNumSet);
  virtual ~WorldCoMConstraint();
  void updateRobotnum(const std::set<int>& model_instance_id);

 protected:
  virtual void evalPositions(KinematicsCache<double>& cache,
                             Eigen::Matrix3Xd& pos, Eigen::MatrixXd& J) const;
  virtual void evalNames(const double* t,
                         std::vector<std::string>& cnst_names) const;

 private:
  static const std::set<int> defaultRobotNumSet;

  std::set<int> m_model_instance_id_;
};

class DRAKE_EXPORT RelativePositionConstraint
    : public PositionConstraint {
 public:
  RelativePositionConstraint(RigidBodyTree* model, const Eigen::Matrix3Xd& pts,
                             const Eigen::MatrixXd& lb,
                             const Eigen::MatrixXd& ub, int bodyA_idx,
                             int bodyB_idx,
                             const Eigen::Matrix<double, 7, 1>& bTbp,
                             const Eigen::Vector2d& tspan);
  virtual ~RelativePositionConstraint();

 protected:
  virtual void evalPositions(KinematicsCache<double>& cache,
                             Eigen::Matrix3Xd& pos, Eigen::MatrixXd& J) const;
  virtual void evalNames(const double* t,
                         std::vector<std::string>& cnst_names) const;

 private:
  int bodyA_idx_{};
  int bodyB_idx_{};
  std::string bodyA_name_;
  std::string bodyB_name_;
  Eigen::Isometry3d bpTb_;
};

class DRAKE_EXPORT QuatConstraint
    : public SingleTimeKinematicConstraint {
 public:
  QuatConstraint(
      RigidBodyTree* model, double tol,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~QuatConstraint();
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 protected:
  virtual void evalOrientationProduct(const KinematicsCache<double>& cache,
                                      double& prod,
                                      Eigen::MatrixXd& dprod) const = 0;
 private:
  double tol_{};
};

class DRAKE_EXPORT WorldQuatConstraint
    : public QuatConstraint {
 public:
  WorldQuatConstraint(
      RigidBodyTree* model, int body, const Eigen::Vector4d& quat_des,
      double tol,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldQuatConstraint();
  virtual void name(const double* t, std::vector<std::string>& name_str) const;

 protected:
  virtual void evalOrientationProduct(const KinematicsCache<double>& cache,
                                      double& prod,
                                      Eigen::MatrixXd& dprod) const;
 private:
  int body_{};
  std::string body_name_;
  Eigen::Vector4d quat_des_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKE_EXPORT RelativeQuatConstraint
    : public QuatConstraint {
 public:
  RelativeQuatConstraint(
      RigidBodyTree* model, int bodyA_idx, int bodyB_idx,
      const Eigen::Vector4d& quat_des, double tol,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual void name(const double* t, std::vector<std::string>& name_str) const;
  virtual ~RelativeQuatConstraint();

 protected:
  virtual void evalOrientationProduct(const KinematicsCache<double>& cache,
                                      double& prod,
                                      Eigen::MatrixXd& dprod) const;
 private:
  int bodyA_idx_{};
  int bodyB_idx_{};
  std::string bodyA_name_;
  std::string bodyB_name_;
  Eigen::Vector4d quat_des_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKE_EXPORT EulerConstraint
    : public SingleTimeKinematicConstraint {
 public:
  EulerConstraint(
      RigidBodyTree* model, const Eigen::Vector3d& lb,
      const Eigen::Vector3d& ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~EulerConstraint(void) {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 protected:
  virtual void evalrpy(const KinematicsCache<double>& cache,
                       Eigen::Vector3d& rpy, Eigen::MatrixXd& J) const = 0;
  bool null_constraint_row(int row) const { return null_constraint_rows_[row]; }

 private:
  Eigen::VectorXd ub_;
  Eigen::VectorXd lb_;
  bool null_constraint_rows_[3];
  Eigen::VectorXd avg_rpy_;
};

class DRAKE_EXPORT WorldEulerConstraint
    : public EulerConstraint {
 public:
  WorldEulerConstraint(
      RigidBodyTree* model, int body, const Eigen::Vector3d& lb,
      const Eigen::Vector3d& ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldEulerConstraint();
  virtual void name(const double* t, std::vector<std::string>& name_str) const;

 protected:
  virtual void evalrpy(const KinematicsCache<double>& cache,
                       Eigen::Vector3d& rpy, Eigen::MatrixXd& J) const;

 private:
  int body_{};
  std::string body_name_;
};

class DRAKE_EXPORT GazeConstraint
    : public SingleTimeKinematicConstraint {
 public:
  GazeConstraint(
      RigidBodyTree* model, const Eigen::Vector3d& axis,
      double conethreshold = 0.0,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~GazeConstraint(void) {}

 protected:
  double get_conethreshold() const { return conethreshold_; }
  const Eigen::Vector3d& get_axis() const { return axis_; }

 private:
  Eigen::Vector3d axis_;
  double conethreshold_{};

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKE_EXPORT GazeOrientConstraint
    : public GazeConstraint {
 public:
  GazeOrientConstraint(
      RigidBodyTree* model, const Eigen::Vector3d& axis,
      const Eigen::Vector4d& quat_des, double conethreshold, double threshold,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~GazeOrientConstraint(void) {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 protected:
  virtual void evalOrientation(const KinematicsCache<double>& cache,
                               Eigen::Vector4d& quat,
                               Eigen::MatrixXd& dquat_dq) const = 0;
 private:
  double threshold_{};
  Eigen::Vector4d quat_des_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKE_EXPORT WorldGazeOrientConstraint
    : public GazeOrientConstraint {
 public:
  WorldGazeOrientConstraint(
      RigidBodyTree* model, int body, const Eigen::Vector3d& axis,
      const Eigen::Vector4d& quat_des, double conethreshold, double threshold,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldGazeOrientConstraint() {}
  virtual void name(const double* t, std::vector<std::string>& name_str) const;

 protected:
  virtual void evalOrientation(const KinematicsCache<double>& cache,
                               Eigen::Vector4d& quat,
                               Eigen::MatrixXd& dquat_dq) const;
 private:
  int body_{};
  std::string body_name_;
};

class DRAKE_EXPORT GazeDirConstraint
    : public GazeConstraint {
 public:
  GazeDirConstraint(
      RigidBodyTree* model, const Eigen::Vector3d& axis,
      const Eigen::Vector3d& dir, double conethreshold,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~GazeDirConstraint(void) {}
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 protected:
  const Eigen::Vector3d& get_dir() const { return dir_; }

 private:
  Eigen::Vector3d dir_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKE_EXPORT WorldGazeDirConstraint
    : public GazeDirConstraint {
 public:
  WorldGazeDirConstraint(
      RigidBodyTree* model, int body, const Eigen::Vector3d& axis,
      const Eigen::Vector3d& dir, double conethreshold,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldGazeDirConstraint(void) {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void name(const double* t, std::vector<std::string>& name_str) const;

 private:
  int body_{};
  std::string body_name_;
};

class DRAKE_EXPORT GazeTargetConstraint
    : public GazeConstraint {
 public:
  GazeTargetConstraint(
      RigidBodyTree* model, const Eigen::Vector3d& axis,
      const Eigen::Vector3d& target, const Eigen::Vector3d& gaze_origin,
      double conethreshold,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~GazeTargetConstraint(void) {}
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 protected:
  const Eigen::Vector3d& get_target() const { return target_; }
  const Eigen::Vector3d& get_gaze_origin() const { return gaze_origin_; }

 private:
  Eigen::Vector3d target_;
  Eigen::Vector3d gaze_origin_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKE_EXPORT WorldGazeTargetConstraint
    : public GazeTargetConstraint {
 public:
  WorldGazeTargetConstraint(
      RigidBodyTree* model, int body, const Eigen::Vector3d& axis,
      const Eigen::Vector3d& target, const Eigen::Vector3d& gaze_origin,
      double conethreshold,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldGazeTargetConstraint(void) {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void name(const double* t, std::vector<std::string>& name_str) const;

 private:
  int body_{};
  std::string body_name_;
};

class DRAKE_EXPORT RelativeGazeTargetConstraint
    : public GazeTargetConstraint {
 public:
  RelativeGazeTargetConstraint(
      RigidBodyTree* model, int bodyA_idx, int bodyB_idx,
      const Eigen::Vector3d& axis, const Eigen::Vector3d& target,
      const Eigen::Vector3d& gaze_origin, double conethreshold,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~RelativeGazeTargetConstraint(void) {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void name(const double* t, std::vector<std::string>& name_str) const;

 private:
  int bodyA_idx_{};
  int bodyB_idx_{};
  std::string bodyA_name_;
  std::string bodyB_name_;
};

class DRAKE_EXPORT RelativeGazeDirConstraint
    : public GazeDirConstraint {
 public:
  RelativeGazeDirConstraint(
      RigidBodyTree* model, int bodyA_idx, int bodyB_idx,
      const Eigen::Vector3d& axis, const Eigen::Vector3d& dir,
      double conethreshold,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~RelativeGazeDirConstraint(void) {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void name(const double* t, std::vector<std::string>& name_str) const;

 private:
  int bodyA_idx_{};
  int bodyB_idx_{};
  std::string bodyA_name_;
  std::string bodyB_name_;
};

class DRAKE_EXPORT Point2PointDistanceConstraint
    : public SingleTimeKinematicConstraint {
 public:
  Point2PointDistanceConstraint(
      RigidBodyTree* model, int bodyA, int bodyB, const Eigen::Matrix3Xd& ptA,
      const Eigen::Matrix3Xd& ptB, const Eigen::VectorXd& lb,
      const Eigen::VectorXd& ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~Point2PointDistanceConstraint(void) {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void name(const double* t, std::vector<std::string>& name_str) const;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 private:
  int bodyA_{};
  int bodyB_{};
  Eigen::Matrix3Xd ptA_;
  Eigen::Matrix3Xd ptB_;
  Eigen::VectorXd dist_lb_;
  Eigen::VectorXd dist_ub_;
};

class DRAKE_EXPORT Point2LineSegDistConstraint
    : public SingleTimeKinematicConstraint {
 public:
  Point2LineSegDistConstraint(
      RigidBodyTree* model, int pt_body, const Eigen::Vector3d& pt,
      int line_body, const Eigen::Matrix<double, 3, 2>& line_ends,
      double dist_lb, double dist_ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~Point2LineSegDistConstraint(void) {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void name(const double* t, std::vector<std::string>& name_str) const;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 private:
  int pt_body_{};
  Eigen::Vector3d pt_;
  int line_body_{};
  Eigen::Matrix3Xd line_ends_;
  double dist_lb_{};
  double dist_ub_{};

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKE_EXPORT WorldFixedPositionConstraint
    : public MultipleTimeKinematicConstraint {
 public:
  WorldFixedPositionConstraint(
      RigidBodyTree* model, int body, const Eigen::Matrix3Xd& pts,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldFixedPositionConstraint(void) {}
  virtual int getNumConstraint(const double* t, int n_breaks) const;
  virtual void eval_valid(const double* valid_t, int num_valid_t,
                          const Eigen::MatrixXd& valid_q, Eigen::VectorXd& c,
                          Eigen::MatrixXd& dc_valid) const;
  virtual void bounds(const double* t, int n_breaks, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;
  virtual void name(const double* t, int n_breaks,
                    std::vector<std::string>& name_str) const;

 private:
  int body_{};
  std::string body_name_;
  Eigen::Matrix3Xd pts_;
};

class DRAKE_EXPORT WorldFixedOrientConstraint
    : public MultipleTimeKinematicConstraint {
 public:
  WorldFixedOrientConstraint(
      RigidBodyTree* model, int body,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual int getNumConstraint(const double* t, int n_breaks) const;
  virtual ~WorldFixedOrientConstraint(void) {}
  virtual void eval_valid(const double* valid_t, int num_valid_t,
                          const Eigen::MatrixXd& valid_q, Eigen::VectorXd& c,
                          Eigen::MatrixXd& dc_valid) const;
  virtual void bounds(const double* t, int n_breaks, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;
  virtual void name(const double* t, int n_breaks,
                    std::vector<std::string>& name_str) const;

 private:
  int body_{};
  std::string body_name_;
};

class DRAKE_EXPORT WorldFixedBodyPoseConstraint
    : public MultipleTimeKinematicConstraint {
 public:
  WorldFixedBodyPoseConstraint(
      RigidBodyTree* model, int body,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldFixedBodyPoseConstraint(void) {}
  virtual int getNumConstraint(const double* t, int n_breaks) const;
  virtual void eval_valid(const double* valid_t, int num_valid_t,
                          const Eigen::MatrixXd& valid_q, Eigen::VectorXd& c,
                          Eigen::MatrixXd& dc_valid) const;
  virtual void bounds(const double* t, int n_breaks, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;
  virtual void name(const double* t, int n_breaks,
                    std::vector<std::string>& name_str) const;

 private:
  int body_{};
  std::string body_name_;
};

class DRAKE_EXPORT AllBodiesClosestDistanceConstraint
    : public SingleTimeKinematicConstraint {
 public:
  AllBodiesClosestDistanceConstraint(
      RigidBodyTree* model, double lb, double ub,
      const std::vector<int>& active_bodies_idx,
      const std::set<std::string>& active_group_names,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~AllBodiesClosestDistanceConstraint() {}
  virtual void updateRobot(RigidBodyTree* robot);
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void name(const double* t, std::vector<std::string>& name) const;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 private:
  double ub_{};
  double lb_{};
  std::vector<int> active_bodies_idx_;
  std::set<std::string> active_group_names_;
};

class DRAKE_EXPORT MinDistanceConstraint
    : public SingleTimeKinematicConstraint {
 public:
  MinDistanceConstraint(
      RigidBodyTree* model, double min_distance,
      const std::vector<int>& active_bodies_idx,
      const std::set<std::string>& active_group_names,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~MinDistanceConstraint() {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void name(const double* t, std::vector<std::string>& name) const;
  void scaleDistance(const Eigen::VectorXd& dist, Eigen::VectorXd& scaled_dist,
                     Eigen::MatrixXd& dscaled_dist_ddist) const;
  void penalty(const Eigen::VectorXd& dist, Eigen::VectorXd& cost,
               Eigen::MatrixXd& dcost_ddist) const;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 private:
  double min_distance_{};
  std::vector<int> active_bodies_idx_;
  std::set<std::string> active_group_names_;
};

class DRAKE_EXPORT WorldPositionInFrameConstraint
    : public WorldPositionConstraint {
 public:
  WorldPositionInFrameConstraint(
      RigidBodyTree* model, int body, const Eigen::Matrix3Xd& pts,
      const Eigen::Matrix4d& T_world_to_frame, const Eigen::MatrixXd& lb,
      const Eigen::MatrixXd& ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~WorldPositionInFrameConstraint();

 protected:
  virtual void evalPositions(KinematicsCache<double>& cache,
                             Eigen::Matrix3Xd& pos, Eigen::MatrixXd& J) const;
  virtual void evalNames(const double* t,
                         std::vector<std::string>& cnst_names) const;

 private:
  Eigen::Matrix4d T_world_to_frame_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKE_EXPORT PostureChangeConstraint
    : public MultipleTimeLinearPostureConstraint {
 public:
  PostureChangeConstraint(
      RigidBodyTree* model, const Eigen::VectorXi& joint_ind,
      const Eigen::VectorXd& lb_change, const Eigen::VectorXd& ub_change,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~PostureChangeConstraint() {}
  virtual int getNumConstraint(const double* t, int n_breaks) const;
  virtual void feval(const double* t, int n_breaks, const Eigen::MatrixXd& q,
                     Eigen::VectorXd& c) const;
  virtual void geval(const double* t, int n_breaks, Eigen::VectorXi& iAfun,
                     Eigen::VectorXi& jAvar, Eigen::VectorXd& A) const;
  virtual void name(const double* t, int n_breaks,
                    std::vector<std::string>& name_str) const;
  virtual void bounds(const double* t, int n_breaks, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 protected:
  virtual void setJointChangeBounds(const Eigen::VectorXi& joint_ind,
                                    const Eigen::VectorXd& lb_change,
                                    const Eigen::VectorXd& ub_change);

 private:
  Eigen::VectorXi joint_ind_;
  Eigen::VectorXd lb_change_;
  Eigen::VectorXd ub_change_;
};

class DRAKE_EXPORT GravityCompensationTorqueConstraint
    : public SingleTimeKinematicConstraint {
 public:
  GravityCompensationTorqueConstraint(
      RigidBodyTree* model, const Eigen::VectorXi& joint_indices,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const Eigen::Vector2d& tspan = DrakeRigidBodyConstraint::default_tspan);
  virtual ~GravityCompensationTorqueConstraint() {}
  virtual void eval(const double* t, KinematicsCache<double>& cache,
                    Eigen::VectorXd& c, Eigen::MatrixXd& dc) const;
  virtual void name(const double* t, std::vector<std::string>& name) const;
  virtual void bounds(const double* t, Eigen::VectorXd& lb,
                      Eigen::VectorXd& ub) const;

 private:
  Eigen::VectorXi joint_indices_;
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
};
