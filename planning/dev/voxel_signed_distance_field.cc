#include "drake/planning/dev/voxel_signed_distance_field.h"

#include <utility>

#include <voxelized_geometry_tools/signed_distance_field.hpp>

#include "drake/planning/dev/voxel_signed_distance_field_internal.h"

namespace drake {
namespace planning {

using voxelized_geometry_tools::SignedDistanceField;

VoxelSignedDistanceField::VoxelSignedDistanceField() {
  auto internal_sdf = std::make_shared<SignedDistanceField<float>>();
  internal_representation_ =
      std::shared_ptr<void>(internal_sdf, internal_sdf.get());
}

bool VoxelSignedDistanceField::is_empty() const {
  const auto& internal_sdf = internal::GetInternalSignedDistanceField(*this);
  return !internal_sdf.IsInitialized();
}

VoxelSignedDistanceField::VoxelSignedDistanceField(
    std::shared_ptr<void> internal_representation) {
  DRAKE_THROW_UNLESS(internal_representation != nullptr);
  internal_representation_ = std::move(internal_representation);
}

}  // namespace planning
}  // namespace drake
