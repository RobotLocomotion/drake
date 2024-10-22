#include "drake/planning/dev/voxel_tagged_object_collision_map.h"

#include <utility>

#include <voxelized_geometry_tools/signed_distance_field.hpp>
#include <voxelized_geometry_tools/tagged_object_collision_map.hpp>

#include "drake/planning/dev/voxel_signed_distance_field_internal.h"
#include "drake/planning/dev/voxel_tagged_object_collision_map_internal.h"

namespace drake {
namespace planning {

using voxelized_geometry_tools::SignedDistanceField;
using voxelized_geometry_tools::TaggedObjectCollisionMap;

namespace {

std::shared_ptr<void> CopyInternalRepresentation(
    const VoxelTaggedObjectCollisionMap& collision_map) {
  auto copied_internal_collision_map =
      std::make_shared<TaggedObjectCollisionMap>(
          internal::GetInternalTaggedObjectCollisionMap(collision_map));
  return std::shared_ptr<void>(
      copied_internal_collision_map, copied_internal_collision_map.get());
}

}  // namespace

VoxelTaggedObjectCollisionMap::VoxelTaggedObjectCollisionMap() {
  InitializeEmpty();
}

VoxelTaggedObjectCollisionMap::VoxelTaggedObjectCollisionMap(
    const VoxelTaggedObjectCollisionMap& other) {
  internal_representation_ = CopyInternalRepresentation(other);
}

VoxelTaggedObjectCollisionMap::VoxelTaggedObjectCollisionMap(
    VoxelTaggedObjectCollisionMap&& other) {
  internal_representation_ = std::move(other.internal_representation_);
  other.InitializeEmpty();
}

VoxelTaggedObjectCollisionMap& VoxelTaggedObjectCollisionMap::operator=(
    const VoxelTaggedObjectCollisionMap& other) {
  if (this != &other) {
    internal_representation_ = CopyInternalRepresentation(other);
  }
  return *this;
}

VoxelTaggedObjectCollisionMap& VoxelTaggedObjectCollisionMap::operator=(
    VoxelTaggedObjectCollisionMap&& other) {
  if (this != &other) {
    internal_representation_ = std::move(other.internal_representation_);
    other.InitializeEmpty();
  }
  return *this;
}

VoxelSignedDistanceField
VoxelTaggedObjectCollisionMap::ExportSignedDistanceField(
    const std::vector<uint32_t>& objects_to_include,
    const VoxelSignedDistanceField::GenerationParameters& parameters) const {
  const auto& internal_collision_map =
      internal::GetInternalTaggedObjectCollisionMap(*this);

  auto internal_sdf = std::make_shared<SignedDistanceField<float>>(
      internal_collision_map.ExtractSignedDistanceFieldFloat(
          objects_to_include, internal::ToVGT(parameters)));

  auto internal_sdf_representation =
      std::shared_ptr<void>(internal_sdf, internal_sdf.get());

  return VoxelSignedDistanceField(internal_sdf_representation);
}

bool VoxelTaggedObjectCollisionMap::is_empty() const {
  const auto& internal_collision_map =
      internal::GetInternalTaggedObjectCollisionMap(*this);
  return !internal_collision_map.IsInitialized();
}

void VoxelTaggedObjectCollisionMap::InitializeEmpty() {
  auto internal_collision_map = std::make_shared<TaggedObjectCollisionMap>();
  internal_representation_ = std::shared_ptr<void>(
      internal_collision_map, internal_collision_map.get());
}

}  // namespace planning
}  // namespace drake
