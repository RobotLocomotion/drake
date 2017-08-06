#pragma once

#include <memory>

#include "drake/multibody/collision/model.h"

namespace drake {
namespace multibody {
namespace collision {

enum class ModelType {
  kUnusable = 0,
#ifndef DRAKE_DISABLE_FCL
  kFcl = 1,
#endif
#ifdef BULLET_COLLISION
  kBullet = 2
#endif
};

/// Returns a unique pointer to a Model that uses the collision backend
/// specified by @p type.
#ifdef BULLET_COLLISION
std::unique_ptr<Model> newModel(ModelType type = ModelType::kBullet);
#else
std::unique_ptr<Model> newModel(ModelType type = ModelType::kUnusable);
#endif

}  // namespace collision
}  // namespace multibody
}  // namespace drake
