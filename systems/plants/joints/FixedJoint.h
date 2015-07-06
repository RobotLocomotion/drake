/*
 * FixedJoint.h
 *
 *  Created on: Mar 26, 2015
 *      Author: twan
 */

#ifndef DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_
#define DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_

#include "DrakeJoint.h"

class DLLEXPORT_DRAKEJOINT FixedJoint: public DrakeJoint
{
public:
  FixedJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body);

  virtual ~FixedJoint();

  virtual std::string getPositionName(int index) const { throw std::runtime_error("bad index"); }

  virtual Eigen::Isometry3d jointTransform(const Eigen::Ref<const Eigen::VectorXd>& q) const;

  virtual void motionSubspace(const Eigen::Ref<const Eigen::VectorXd>& q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace = nullptr) const;

  virtual void motionSubspaceDotTimesV(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v, Vector6d& motion_subspace_dot_times_v,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq = nullptr,
      Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr) const;

  virtual Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const;

  virtual void qdot2v(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const;

  virtual void v2qdot(const Eigen::Ref<const Eigen::VectorXd>& q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const;
};

#endif /* DRAKE_SYSTEMS_PLANTS_JOINTS_FIXEDJOINT_H_ */
