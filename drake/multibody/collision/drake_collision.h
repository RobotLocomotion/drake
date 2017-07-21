#pragma once

#include <memory>

#include "drake/multibody/collision/model.h"

namespace drake {
namespace multibody {
namespace collision {
std::unique_ptr<Model> newModel();
}  // namespace collision
}  // namespace multibody
}  // namespace drake
