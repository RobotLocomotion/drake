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
  double joint_limit_min;
  double joint_limit_max;
  double damping;
  double coulomb_friction;
  double coulomb_window;

protected:
  FixedAxisOneDoFJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Matrix<double, TWIST_SIZE, 1>& joint_axis);

public:
  virtual ~FixedAxisOneDoFJoint();

  void setJointLimits(double joint_limit_min, double joint_limit_max);
  double getJointLimitMin(void) const { return joint_limit_min; }
  double getJointLimitMax(void) const { return joint_limit_max; }

  virtual void motionSubspace(const Eigen::Ref<const Eigen::VectorXd>& q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace) const; //override;

  virtual void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v, Vector6d& motion_subspace_dot_times_v,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq = nullptr,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr) const; //override;

  virtual void randomConfiguration(Eigen::Ref<Eigen::VectorXd>& q, std::default_random_engine& generator) const; //override;

  virtual void qdot2v(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const; //override;

  virtual void v2qdot(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const; //override;

  void setDynamics(double damping, double coulomb_friction, double coulomb_window);
//  GradientVar<double,1,1> computeFrictionForce(const Eigen::Ref<const VectorXd>& v, int gradient_order) const;

  virtual void setupOldKinematicTree(RigidBodyManipulator* model, int body_ind, int position_num_start, int velocity_num_start) const;
};

#endif /* ONEDOFJOINT_H_ */
