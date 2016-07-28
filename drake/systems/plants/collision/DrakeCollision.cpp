#include "drake/systems/plants/collision/DrakeCollision.h"

#include <iostream>

#include "drake/common/drake_assert.h"
#ifdef BULLET_COLLISION
#include "drake/systems/plants/collision/bullet_model.h"
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
  DRAKE_ABORT_UNLESS(!"DrakeCollision must be compiled with Bullet.");
  // This return statement is actually never reached when 
  // BULLET_COLLISION is defined.
  // It is placed here to avoid a compiler error on not having a return
  // statement.
  return unique_ptr<Model>();
#endif
}
};
