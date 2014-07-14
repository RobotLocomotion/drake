#ifndef PRISMATICJOINT_H_
#define PRISMATICJOINT_H_

#include "OneDoFJoint.h"

class PrismaticJoint: public OneDoFJoint
{
private:
  e::Vector3d translation_axis;
public:
  PrismaticJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, const e::Vector3d& translation_axis);

  virtual ~PrismaticJoint();

  virtual e::AffineCompact3d jointTransform(double* const q) const override;

private:
  static e::Matrix<double, TWIST_SIZE, 1> spatialJointAxis(e::Vector3d translation_axis);
};

#endif /* PRISMATICJOINT_H_ */
