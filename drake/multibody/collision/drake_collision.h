#pragma once

#include <memory>

#include "drake/multibody/collision/model.h"

namespace drake {
namespace multibody {
namespace collision {

enum ModelType {
  kUnusable = 0,
#ifndef DRAKE_DISABLE_FCL
  kFcl = 1,
#endif
#ifdef BULLET_COLLISION
  kBullet = 2
#endif
};

#ifdef BULLET_COLLISION
std::unique_ptr<Model> newModel(ModelType type = kBullet);
#else
std::unique_ptr<Model> newModel(ModelType type = kUnusable);
#endif

}  // namespace collision
}  // namespace multibody
}  // namespace drake
