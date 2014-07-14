#ifndef DRAKEJOINTBASEIMPL_H_
#define DRAKEJOINTBASEIMPL_H_

#include "DrakeJoint.h"

template<int NumPositions, int NumVelocities>
class DrakeJointBaseImpl: public DrakeJoint
{
public:
  static const int NUM_POSITIONS = NumPositions;
  static const int NUM_VELOCITIES = NumVelocities;

public:
  DrakeJointBaseImpl();
  virtual ~DrakeJointBaseImpl();
};

#endif /* DRAKEJOINTBASEIMPL_H_ */
