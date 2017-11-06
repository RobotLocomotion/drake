#pragma once

#include <map>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {

/// This class provides a representation of a `<model>` entry within an SDF
/// file.
class SDFModel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SDFModel)

  /// Creates a new SDF model object specification with the given `model_name`.
  /// Per SDF specification, `model_name` must not match any other model name in
  /// the world (<world>).
  explicit SDFModel(const std::string& model_name) : name_(model_name) {}

  /// Returns the name of `this` link.
  const std::string& name() const { return name_; }

 private:
  // Name of the root frame of this cache.
  std::string name_;
};

}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
