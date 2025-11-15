#pragma once

#include <limits>
#include <memory>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/parallelism.h"

namespace drake {
namespace planning {
// Forward declarations.
class VoxelOccupancyMap;
class VoxelTaggedObjectOccupancyMap;

/// Container for voxelized signed distance fields. To enable efficient sharing
/// signed distance fields (which may be quite large) between multiple uses, a
/// VoxelSignedDistanceField operates equivalently to shared_ptr<const T> for
/// the underlying voxelized signed distance field.
class VoxelSignedDistanceField {
 public:
  /// Param struct for generating a VoxelSignedDistanceField.
  struct GenerationParameters {
    /// Out-of-bounds value stored in the VoxelSignedDistanceField.
    float oob_value = std::numeric_limits<float>::infinity();
    /// Parallelism to use when generating the VoxelSignedDistanceField.
    Parallelism parallelism = Parallelism::None();
    /// If cells with unknown occupancy (i.e. 0.5) should be treated as filled
    /// (i.e. occupancy > 0.5) or empty (i.e. occupancy < 0.5) for the purposes
    /// of generating the VoxelSignedDistanceField.
    bool unknown_is_filled = true;
    /// Should a "virtual border" be added to the occpancy grid, such that all
    /// cells at the edges of the occupancy grid are assumed to be on the border
    /// of filled/empty space? This should only be enabled for specific uses,
    /// such as certain grasp search problems.
    bool add_virtual_border = false;
  };

  /// Default constructor creates an empty VoxelSignedDistanceField.
  VoxelSignedDistanceField();

  /// VoxelSignedDistanceField provides copy, move, and assignment.
  VoxelSignedDistanceField(const VoxelSignedDistanceField& other);
  VoxelSignedDistanceField(VoxelSignedDistanceField&& other);
  VoxelSignedDistanceField& operator=(const VoxelSignedDistanceField& other);
  VoxelSignedDistanceField& operator=(VoxelSignedDistanceField&& other);

  /// Get the name of the parent body frame.
  const std::string& parent_body_name() const;

  /// Returns true if empty. A VoxelSignedDistanceField will be empty if it is
  /// in the default constructed state or has been moved-from.
  bool is_empty() const;

  // Internal-only, access to the internal voxel grid.
  const void* internal_representation() const {
    DRAKE_DEMAND(internal_representation_ != nullptr);
    return internal_representation_.get();
  }

 private:
  void InitializeEmpty();

  friend class VoxelOccupancyMap;
  friend class VoxelTaggedObjectOccupancyMap;

  // Construct from an existing internal representation, used only by friends.
  explicit VoxelSignedDistanceField(
      std::shared_ptr<void> internal_representation);

  std::shared_ptr<void> internal_representation_;
};

}  // namespace planning
}  // namespace drake
