#include "drake/multibody/collision/drake_collision.h"

#include "drake/common/drake_assert.h"
#ifdef BULLET_COLLISION
#include "drake/multibody/collision/bullet_model.h"
#endif
#include "drake/multibody/collision/collision_filter.h"
#ifndef DRAKE_DISABLE_FCL
#include "drake/multibody/collision/fcl_model.h"
#endif
#include "drake/multibody/collision/unusable_model.h"

using std::unique_ptr;

namespace drake {
namespace multibody {
namespace collision {

unique_ptr<Model> newModel(ModelType type) {
  switch (type) {
    case (ModelType::kUnusable): {
      return unique_ptr<Model>(new UnusableModel());
    }
#ifndef DRAKE_DISABLE_FCL
    case (ModelType::kFcl): {
      return unique_ptr<Model>(new FclModel());
    }
#endif
#ifdef BULLET_COLLISION
    case (ModelType::kBullet): {
      return unique_ptr<Model>(new BulletModel());
    }
#endif
  }
  DRAKE_UNREACHABLE();
}

}  // namespace collision
}  // namespace multibody
}  // namespace drake
