#ifndef DRAKEJOINT_H_
#define DRAKEJOINT_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "RigidBody.h"

class RigidBody;

namespace e = Eigen;

class DrakeJoint
{
public:
  static const int TWIST_SIZE = 6; // TODO: move to a better place

private:
  const RigidBody& parent_body;
  const e::AffineCompact3d transform_to_parent_body;

protected:
  DrakeJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body);

public:
  virtual ~DrakeJoint();

  const RigidBody& getParentBody() const;

  const e::AffineCompact3d& getTransformToParentBody() const;

  template <typename DerivedA, typename DerivedB>
  virtual void motionSubspace(double* const q, MatrixBase<DerivedA>& motion_subspace, MatrixBase<DerivedB>* dmotion_subspace = nullptr) const = 0;

  virtual e::AffineCompact3d jointTransform(double* const q) const = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif /* DRAKEJOINT_H_ */
