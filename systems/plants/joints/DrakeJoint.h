#ifndef DRAKEJOINT_H_
#define DRAKEJOINT_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include "RigidBody.h"
#include <random>
#include "drakeGeometryUtil.h"

class RigidBody;

class DrakeJoint
{
  // disable copy construction and assignment
  DrakeJoint(const DrakeJoint&) = delete;
  DrakeJoint& operator=(const DrakeJoint&) = delete;

private:
  const std::string name;
  const Eigen::Isometry3d transform_to_parent_body;
  const int num_positions;
  const int num_velocities;

protected:
  typedef Eigen::Matrix<double, 6, 1> Vector6d;


public:
  DrakeJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, int num_positions, int num_velocities);
  typedef Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> MotionSubspaceType;

  virtual ~DrakeJoint();

  const Eigen::Isometry3d& getTransformToParentBody() const;

  const int getNumPositions() const;

  const int getNumVelocities() const;

  const std::string& getName() const;

  virtual Eigen::Isometry3d jointTransform(double* const q) const = 0;

  virtual void motionSubspace(double* const q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace = nullptr) const = 0;

  virtual void motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v,
      typename Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq = nullptr,
      typename Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr) const = 0;

  virtual void randomConfiguration(double* q, std::default_random_engine& generator) const = 0;

  virtual void qdot2v(double* q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const = 0;

  virtual void v2qdot(double* q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const = 0;

//public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif /* DRAKEJOINT_H_ */
