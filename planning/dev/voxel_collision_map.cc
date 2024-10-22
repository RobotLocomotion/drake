#include "drake/planning/dev/voxel_collision_map.h"

#include <utility>

#include <voxelized_geometry_tools/collision_map.hpp>
#include <voxelized_geometry_tools/signed_distance_field.hpp>

#include "drake/planning/dev/voxel_collision_map_internal.h"
#include "drake/planning/dev/voxel_signed_distance_field_internal.h"

namespace drake {
namespace planning {

using voxelized_geometry_tools::CollisionMap;
using voxelized_geometry_tools::SignedDistanceField;

namespace {

std::shared_ptr<void> CopyInternalRepresentation(
    const VoxelCollisionMap& collision_map) {
  auto copied_internal_collision_map = std::make_shared<CollisionMap>(
      internal::GetInternalCollisionMap(collision_map));
  return std::shared_ptr<void>(
      copied_internal_collision_map, copied_internal_collision_map.get());
}

}  // namespace

VoxelCollisionMap::VoxelCollisionMap() {
  InitializeEmpty();
}

VoxelCollisionMap::VoxelCollisionMap(const VoxelCollisionMap& other) {
  internal_representation_ = CopyInternalRepresentation(other);
}

VoxelCollisionMap::VoxelCollisionMap(VoxelCollisionMap&& other) {
  internal_representation_ = std::move(other.internal_representation_);
  other.InitializeEmpty();
}

VoxelCollisionMap& VoxelCollisionMap::operator=(
    const VoxelCollisionMap& other) {
  if (this != &other) {
    internal_representation_ = CopyInternalRepresentation(other);
  }
  return *this;
}

VoxelCollisionMap& VoxelCollisionMap::operator=(VoxelCollisionMap&& other) {
  if (this != &other) {
    internal_representation_ = std::move(other.internal_representation_);
    other.InitializeEmpty();
  }
  return *this;
}

VoxelSignedDistanceField VoxelCollisionMap::ExportSignedDistanceField(
    const VoxelSignedDistanceField::GenerationParameters& parameters) const {
  const auto& internal_collision_map = internal::GetInternalCollisionMap(*this);

  auto internal_sdf = std::make_shared<SignedDistanceField<float>>(
      internal_collision_map.ExtractSignedDistanceFieldFloat(
          internal::ToVGT(parameters)));

  auto internal_sdf_representation =
      std::shared_ptr<void>(internal_sdf, internal_sdf.get());

  return VoxelSignedDistanceField(internal_sdf_representation);
}

bool VoxelCollisionMap::is_empty() const {
  const auto& internal_collision_map = internal::GetInternalCollisionMap(*this);
  return !internal_collision_map.IsInitialized();
}

void VoxelCollisionMap::InitializeEmpty() {
  auto internal_collision_map = std::make_shared<CollisionMap>();
  internal_representation_ = std::shared_ptr<void>(
      internal_collision_map, internal_collision_map.get());
}

}  // namespace planning
}  // namespace drake
