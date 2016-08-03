#include "drake/systems/plants/collision/DrakeCollision.h"

#include <iostream>

#ifdef BULLET_COLLISION
#include "drake/systems/plants/collision/bullet_model.h"
#else
#include "drake/systems/plants/collision/unusable_model.h"
#endif

using namespace std;
using namespace Eigen;

namespace DrakeCollision {

const bitmask ALL_MASK(bitmask(0).set());
const bitmask NONE_MASK(0);
const bitmask DEFAULT_GROUP(1);

unique_ptr<Model> newModel() {
#ifdef BULLET_COLLISION
  return unique_ptr<Model>(new BulletModel());
#else
  return unique_ptr<Model>(new UnusableModel());
#endif
}
};
