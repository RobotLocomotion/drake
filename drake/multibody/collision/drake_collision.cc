#include "drake/multibody/collision/drake_collision.h"

#ifdef BULLET_COLLISION
#include "drake/multibody/collision/bullet_model.h"
#else
#include "drake/multibody/collision/unusable_model.h"
#endif

using std::unique_ptr;

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

}  // namespace DrakeCollision
