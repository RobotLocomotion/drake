#include "drake/planning/dev/voxel_signed_distance_field.h"

#include <utility>

#include "drake/common/drake_throw.h"

namespace drake {
namespace planning {

using voxelized_geometry_tools::SignedDistanceField;

VoxelSignedDistanceField::VoxelSignedDistanceField() {
  auto internal_sdf = std::make_shared<SignedDistanceField<float>>();
  internal_representation_ =
      std::shared_ptr<void>(internal_sdf, internal_sdf.get());
}

VoxelSignedDistanceField::VoxelSignedDistanceField(
    const voxelized_geometry_tools::CollisionMap& collision_map,
    const GenerationParameters& generation_params) {
  auto internal_sdf = std::make_shared<SignedDistanceField<float>>(
      collision_map.ExtractSignedDistanceFieldFloat(generation_params));
  internal_representation_ =
      std::shared_ptr<void>(internal_sdf, internal_sdf.get());
}

VoxelSignedDistanceField::VoxelSignedDistanceField(
    const voxelized_geometry_tools::TaggedObjectCollisionMap& collision_map,
    const std::vector<uint32_t>& objects_to_include,
    const GenerationParameters& generation_params) {
  auto internal_sdf = std::make_shared<SignedDistanceField<float>>(
      collision_map.ExtractSignedDistanceFieldFloat(objects_to_include,
                                                    generation_params));
  internal_representation_ =
      std::shared_ptr<void>(internal_sdf, internal_sdf.get());
}

const SignedDistanceField<float>&
VoxelSignedDistanceField::internal_representation() const {
  DRAKE_DEMAND(internal_representation_ != nullptr);
  return *reinterpret_cast<const SignedDistanceField<float>*>(
      internal_representation_.get());
}

}  // namespace planning
}  // namespace drake
