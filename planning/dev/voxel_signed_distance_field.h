#pragma once

#include <limits>
#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/parallelism.h"

namespace drake {
namespace planning {
// Forward declarations.
class VoxelCollisionMap;
class VoxelTaggedObjectCollisionMap;

/// Container for voxelized signed distance fields. To enable efficient sharing
/// signed distance fields (which may be quite large) between multiple uses, a
/// VoxelSignedDistanceField operates equivalently to shared_ptr<const T> for
/// the underlying voxelized signed distance field.
class VoxelSignedDistanceField {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VoxelSignedDistanceField)

  struct GenerationParameters {
    float oob_value = std::numeric_limits<float>::infinity();
    Parallelism parallelism = Parallelism::None();
    bool unknown_is_filled = true;
    bool add_virtual_border = false;
  };

  /// Default constructor.
  VoxelSignedDistanceField();

  bool is_empty() const;

  // Internal-only, access to the internal voxel grid.
  const void* internal_representation() const {
    DRAKE_DEMAND(internal_representation_ != nullptr);
    return internal_representation_.get();
  }

 private:
  friend class VoxelCollisionMap;
  friend class VoxelTaggedObjectCollisionMap;

  // Construct from an existing internal representation, used only by friends.
  VoxelSignedDistanceField(std::shared_ptr<void> internal_representation);

  std::shared_ptr<void> internal_representation_;
};

}  // namespace planning
}  // namespace drake
