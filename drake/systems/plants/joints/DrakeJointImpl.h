#ifndef DRAKE_DRAKEJOINTIMPL_H
#define DRAKE_DRAKEJOINTIMPL_H

#include "DrakeJoint.h"

#if defined(WIN32) || defined(WIN64)
#define OVERRIDE
#else
#define OVERRIDE override
#endif

template <typename Derived>
class DrakeJointImpl : public DrakeJoint
{
private:
  Derived& derived;

public:
  DrakeJointImpl(Derived& derived, const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, int num_positions,
                 int num_velocities) : DrakeJoint(name, transform_to_parent_body, num_positions, num_velocities), derived(derived) { }

  POSITION_AND_VELOCITY_DEPENDENT_METHODS(double, OVERRIDE)
  POSITION_AND_VELOCITY_DEPENDENT_METHODS(Eigen::AutoDiffScalar<Eigen::VectorXd>, OVERRIDE)
};

#undef POSITION_AND_VELOCITY_DEPENDENT_METHODS

#endif //DRAKE_DRAKEJOINTIMPL_H
