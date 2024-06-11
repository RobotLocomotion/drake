#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "voxelized_geometry_tools/collision_map.hpp"
#include "voxelized_geometry_tools/signed_distance_field.hpp"
#include "voxelized_geometry_tools/tagged_object_collision_map.hpp"

#include "drake/common/drake_copyable.h"

namespace drake {
namespace planning {

/// Container for voxelized signed distance fields. To enable efficient sharing
/// signed distance fields (which may be quite large) between multiple uses, a
/// VoxelSignedDistanceField operates equivalently to shared_ptr<const T> for
/// the underlying voxelized signed distand field.
class VoxelSignedDistanceField {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VoxelSignedDistanceField)

  using GenerationParameters =
      voxelized_geometry_tools::SignedDistanceFieldGenerationParameters<float>;

  /// Default constructor.
  VoxelSignedDistanceField();

  /// Construct a voxelized signed distance field from the provided voxel
  /// CollisionMap `collision_map` using the provided parameters
  /// `generation_params`.
  VoxelSignedDistanceField(
      const voxelized_geometry_tools::CollisionMap& collision_map,
      const GenerationParameters& generation_params = {});

  /// Construct a voxelized signed distance field from the provided voxel
  /// TaggedObjectCollisionMap `collision_map` using the provided parameters
  /// `generation_params`. The objects to include are specified by
  /// `objects_to_include`; if left empty all objects will be included.
  VoxelSignedDistanceField(
      const voxelized_geometry_tools::TaggedObjectCollisionMap& collision_map,
      const std::vector<uint32_t>& objects_to_include,
      const GenerationParameters& generation_params = {});

  /// Retrieve the internal representation.
  const voxelized_geometry_tools::SignedDistanceField<float>&
  internal_representation() const;

 private:
  std::shared_ptr<void> internal_representation_;
};

}  // namespace planning
}  // namespace drake
