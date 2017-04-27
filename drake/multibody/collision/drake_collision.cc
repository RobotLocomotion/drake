#include "drake/multibody/collision/drake_collision.h"

#include "drake/multibody/collision/collision_filter.h"

#ifdef BULLET_COLLISION
#include "drake/multibody/collision/bullet_model.h"
#else
#include "drake/multibody/collision/unusable_model.h"
#endif

using std::unique_ptr;

namespace DrakeCollision {

unique_ptr<Model> newModel() {
#ifdef BULLET_COLLISION
  return unique_ptr<Model>(new BulletModel());
#else
  return unique_ptr<Model>(new UnusableModel());
#endif
}

}  // namespace DrakeCollision
