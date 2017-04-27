#pragma once

#include <memory>

#include "drake/multibody/collision/model.h"

namespace DrakeCollision {
std::unique_ptr<Model> newModel();
}  // namespace DrakeCollision
