#include "drake/planning/dev/voxel_signed_distance_field.h"

#include <utility>

#include <voxelized_geometry_tools/signed_distance_field.hpp>

#include "drake/planning/dev/voxel_signed_distance_field_internal.h"

namespace drake {
namespace planning {

using voxelized_geometry_tools::SignedDistanceField;

VoxelSignedDistanceField::VoxelSignedDistanceField() {
  InitializeEmpty();
}

VoxelSignedDistanceField::VoxelSignedDistanceField(
    const VoxelSignedDistanceField& other) = default;

VoxelSignedDistanceField::VoxelSignedDistanceField(
    VoxelSignedDistanceField&& other) {
  internal_representation_ = std::move(other.internal_representation_);
  other.InitializeEmpty();
}

VoxelSignedDistanceField& VoxelSignedDistanceField::operator=(
    const VoxelSignedDistanceField& other) = default;

VoxelSignedDistanceField& VoxelSignedDistanceField::operator=(
    VoxelSignedDistanceField&& other) {
  if (this != &other) {
    internal_representation_ = std::move(other.internal_representation_);
    other.InitializeEmpty();
  }
  return *this;
}

const std::string& VoxelSignedDistanceField::parent_body_name() const {
  const auto& internal_sdf = internal::GetInternalSignedDistanceField(*this);
  return internal_sdf.GetFrame();
}

bool VoxelSignedDistanceField::is_empty() const {
  const auto& internal_sdf = internal::GetInternalSignedDistanceField(*this);
  return !internal_sdf.IsInitialized();
}

void VoxelSignedDistanceField::InitializeEmpty() {
  auto internal_sdf = std::make_shared<SignedDistanceField<float>>();
  internal_representation_ =
      std::shared_ptr<void>(internal_sdf, internal_sdf.get());
}

VoxelSignedDistanceField::VoxelSignedDistanceField(
    std::shared_ptr<void> internal_representation) {
  DRAKE_THROW_UNLESS(internal_representation != nullptr);
  internal_representation_ = std::move(internal_representation);
}

}  // namespace planning
}  // namespace drake
