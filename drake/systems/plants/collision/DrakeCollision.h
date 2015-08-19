#ifndef __DrakeCollision_H__
#define __DrakeCollision_H__

#include <memory>
#include <set>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <stdexcept>
#include <bitset>

//#include "Geometry.h"
#include "PointPair.h"
#include "Element.h"
#include "Model.h"

#include "drakeCollisionMacros.h"

static const int MAX_NUM_COLLISION_FILTER_GROUPS = 128;

namespace DrakeCollision
{

  DLLEXPORT_drakeCollision std::unique_ptr<Model> newModel();

  typedef std::bitset<MAX_NUM_COLLISION_FILTER_GROUPS> bitmask;

  // Constants
  extern const DLLEXPORT_drakeCollision bitmask ALL_MASK;
  extern const DLLEXPORT_drakeCollision bitmask NONE_MASK;
  extern const DLLEXPORT_drakeCollision bitmask DEFAULT_GROUP;

}
#endif

