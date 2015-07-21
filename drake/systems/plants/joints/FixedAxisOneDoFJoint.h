#ifndef ONEDOFJOINT_H_
#define ONEDOFJOINT_H_

#include "DrakeJoint.h"

class DLLEXPORT_DRAKEJOINT FixedAxisOneDoFJoint : public DrakeJoint
{
  // disable copy construction and assignment
  // not available in MSVC2010...
  // FixedAxisOneDoFJoint(const DrakeJoint&) = delete;
  // FixedAxisOneDoFJoint& operator=(const FixedAxisOneDoFJoint&) = delete;

private:
  Eigen::Matrix<double, TWIST_SIZE, 1> joint_axis;
  double damping;
  double coulomb_friction;
  double coulomb_window;

protected:
  FixedAxisOneDoFJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Matrix<double, TWIST_SIZE, 1>& joint_axis);

public:
  virtual ~FixedAxisOneDoFJoint();

  void setJointLimits(double joint_limit_min, double joint_limit_max);

  virtual std::string getPositionName(int index) const { if (index!=0) throw std::runtime_error("bad index"); return name; }

  virtual void motionSubspace(const Eigen::Ref<const Eigen::VectorXd>& q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace) const; //override;

  virtual void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v, Vector6d& motion_subspace_dot_times_v,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq = nullptr,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr) const; //override;

  virtual Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const; //override;

  virtual void qdot2v(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const; //override;

  virtual void v2qdot(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const; //override;

  void setDynamics(double damping, double coulomb_friction, double coulomb_window);

  virtual GradientVar<double, Eigen::Dynamic, 1> frictionTorque(const Eigen::Ref<const Eigen::VectorXd>& v, int gradient_order) const; // override;

};

#endif /* ONEDOFJOINT_H_ */
