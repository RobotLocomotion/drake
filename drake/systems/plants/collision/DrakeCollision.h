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
#include "drake/drakeCollision_export.h"

static const int MAX_NUM_COLLISION_FILTER_GROUPS = 128;

namespace DrakeCollision
{

  DRAKECOLLISION_EXPORT std::unique_ptr<Model> newModel();

  typedef std::bitset<MAX_NUM_COLLISION_FILTER_GROUPS> bitmask;

  // Constants
  extern const DRAKECOLLISION_EXPORT bitmask ALL_MASK;
  extern const DRAKECOLLISION_EXPORT bitmask NONE_MASK;
  extern const DRAKECOLLISION_EXPORT bitmask DEFAULT_GROUP;

}
#endif

