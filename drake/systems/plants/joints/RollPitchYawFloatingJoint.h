#ifndef ROLLPITCHYAWFLOATINGJOINT_H_
#define ROLLPITCHYAWFLOATINGJOINT_H_

#include "DrakeJoint.h"

class DLLEXPORT_DRAKEJOINT RollPitchYawFloatingJoint: public DrakeJoint
{
public:
  // disable copy construction and assignment
  // not available in MSVC2010...
  // RollPitchYawFloatingJoint(const RollPitchYawFloatingJoint&) = delete;
  // RollPitchYawFloatingJoint& operator=(const RollPitchYawFloatingJoint&) = delete;

public:
  RollPitchYawFloatingJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body);

  virtual ~RollPitchYawFloatingJoint();

  virtual std::string getPositionName(int index) const;

  virtual Eigen::Isometry3d jointTransform(const Eigen::Ref<const Eigen::VectorXd>& q) const; //override;

  virtual void motionSubspace(const Eigen::Ref<const Eigen::VectorXd>& q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace = nullptr) const; //override;

  virtual bool isFloating() const { return true; }

  virtual void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v, Vector6d& motion_subspace_dot_times_v,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq = nullptr,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr) const; //override;

  virtual Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const; //override;

  virtual void qdot2v(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const; //override;

  virtual void v2qdot(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const; //override;

};

#endif /* ROLLPITCHYAWFLOATINGJOINT_H_ */
