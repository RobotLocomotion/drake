#ifndef DRAKEJOINT_H_
#define DRAKEJOINT_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "RigidBody.h"

#define TWIST_SIZE (int) 6
typedef Eigen::Matrix<double, TWIST_SIZE, 1> Vector6d;

class RigidBody;

namespace e = Eigen;

class DrakeJoint
{

private:
  const RigidBody& parent_body;
  const e::AffineCompact3d transform_to_parent_body;
  const int num_positions;
  const int num_velocities;

protected:
  DrakeJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, int num_positions, int num_velocities);

public:
  typedef e::Matrix<double, TWIST_SIZE, e::Dynamic> MotionSubspaceType;

  virtual ~DrakeJoint();

  const RigidBody& getParentBody() const;

  const e::AffineCompact3d& getTransformToParentBody() const;

  virtual e::AffineCompact3d jointTransform(double* const q) const = 0;

  virtual void motionSubspace(double* const q, MotionSubspaceType& motion_subspace, e::MatrixXd* dmotion_subspace = nullptr) const = 0;

  virtual void motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v,
      e::Matrix<double, TWIST_SIZE, e::Dynamic>* dmotion_subspace_dot_times_vdq = nullptr,
      e::Matrix<double, TWIST_SIZE, e::Dynamic>* dmotion_subspace_dot_times_vdv = nullptr) const = 0;

  const int getNumPositions() const;

  const int getNumVelocities() const;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif /* DRAKEJOINT_H_ */
