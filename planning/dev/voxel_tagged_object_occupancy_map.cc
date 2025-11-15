#include "drake/planning/dev/voxel_tagged_object_collision_map.h"

#include <utility>

#include <voxelized_geometry_tools/signed_distance_field.hpp>
#include <voxelized_geometry_tools/tagged_object_collision_map.hpp>

#include "drake/planning/dev/voxel_signed_distance_field_internal.h"
#include "drake/planning/dev/voxel_tagged_object_collision_map_internal.h"

namespace drake {
namespace planning {

using common_robotics_utilities::voxel_grid::GridSizes;
using voxelized_geometry_tools::SignedDistanceField;
using voxelized_geometry_tools::TaggedObjectCollisionCell;
using voxelized_geometry_tools::TaggedObjectCollisionMap;

namespace {

std::shared_ptr<void> CopyInternalRepresentation(
    const VoxelTaggedObjectCollisionMap& collision_map) {
  auto copied_internal_collision_map =
      std::make_shared<TaggedObjectCollisionMap>(
          internal::GetInternalTaggedObjectCollisionMap(collision_map));
  return std::shared_ptr<void>(copied_internal_collision_map,
                               copied_internal_collision_map.get());
}

}  // namespace

VoxelTaggedObjectCollisionMap::VoxelTaggedObjectCollisionMap() {
  InitializeEmpty();
}

VoxelTaggedObjectCollisionMap::VoxelTaggedObjectCollisionMap(
    const std::string& parent_body_name, const math::RigidTransformd& X_PG,
    const Eigen::Vector3d& grid_dimensions, const double grid_resolution,
    const float default_occupancy, const uint32_t default_object_id) {
  const GridSizes cru_sizes(grid_resolution, grid_dimensions.x(),
                            grid_dimensions.y(), grid_dimensions.z());
  const TaggedObjectCollisionCell default_cell(default_occupancy,
                                               default_object_id);
  auto internal_collision_map = std::make_shared<TaggedObjectCollisionMap>(
      X_PG.GetAsIsometry3(), parent_body_name, cru_sizes, default_cell);
  internal_representation_ = std::shared_ptr<void>(
      internal_collision_map, internal_collision_map.get());
}

VoxelTaggedObjectCollisionMap::VoxelTaggedObjectCollisionMap(
    const std::string& parent_body_name, const math::RigidTransformd& X_PG,
    const Eigen::Matrix<int64_t, 3, 1>& grid_sizes,
    const double grid_resolution, const float default_occupancy,
    const uint32_t default_object_id) {
  const GridSizes cru_sizes(grid_resolution, grid_sizes.x(), grid_sizes.y(),
                            grid_sizes.z());
  const TaggedObjectCollisionCell default_cell(default_occupancy,
                                               default_object_id);
  auto internal_collision_map = std::make_shared<TaggedObjectCollisionMap>(
      X_PG.GetAsIsometry3(), parent_body_name, cru_sizes, default_cell);
  internal_representation_ = std::shared_ptr<void>(
      internal_collision_map, internal_collision_map.get());
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

const std::string& VoxelTaggedObjectCollisionMap::parent_body_name() const {
  const auto& internal_collision_map =
      internal::GetInternalTaggedObjectCollisionMap(*this);
  return internal_collision_map.GetFrame();
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
