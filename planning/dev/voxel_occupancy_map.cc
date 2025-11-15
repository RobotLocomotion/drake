#include "drake/planning/dev/voxel_occupancy_map.h"

#include <utility>

#include <voxelized_geometry_tools/occupancy_map.hpp>
#include <voxelized_geometry_tools/signed_distance_field.hpp>

#include "drake/planning/dev/voxel_occupancy_map_internal.h"
#include "drake/planning/dev/voxel_signed_distance_field_internal.h"

namespace drake {
namespace planning {

using common_robotics_utilities::voxel_grid::GridSizes;
using voxelized_geometry_tools::OccupancyCell;
using voxelized_geometry_tools::OccupancyMap;
using voxelized_geometry_tools::SignedDistanceField;

namespace {

std::shared_ptr<void> CopyInternalRepresentation(
    const VoxelOccupancyMap& occupancy_map) {
  auto copied_internal_occupancy_map = std::make_shared<OccupancyMap>(
      internal::GetInternalOccupancyMap(occupancy_map));
  return std::shared_ptr<void>(copied_internal_occupancy_map,
                               copied_internal_occupancy_map.get());
}

}  // namespace

VoxelOccupancyMap::VoxelOccupancyMap() {
  InitializeEmpty();
}

VoxelOccupancyMap::VoxelOccupancyMap(const std::string& parent_body_name,
                                     const math::RigidTransformd& X_PG,
                                     const Eigen::Vector3d& grid_dimensions,
                                     const double grid_resolution,
                                     const float default_occupancy) {
  const GridSizes cru_sizes(grid_resolution, grid_dimensions.x(),
                            grid_dimensions.y(), grid_dimensions.z());
  const OccupancyCell default_cell(default_occupancy);
  auto internal_occupancy_map = std::make_shared<OccupancyMap>(
      X_PG.GetAsIsometry3(), parent_body_name, cru_sizes, default_cell);
  internal_representation_ = std::shared_ptr<void>(
      internal_occupancy_map, internal_occupancy_map.get());
}

VoxelOccupancyMap::VoxelOccupancyMap(
    const std::string& parent_body_name, const math::RigidTransformd& X_PG,
    const Eigen::Matrix<int64_t, 3, 1>& grid_sizes,
    const double grid_resolution, const float default_occupancy) {
  const GridSizes cru_sizes(grid_resolution, grid_sizes.x(), grid_sizes.y(),
                            grid_sizes.z());
  const OccupancyCell default_cell(default_occupancy);
  auto internal_occupancy_map = std::make_shared<OccupancyMap>(
      X_PG.GetAsIsometry3(), parent_body_name, cru_sizes, default_cell);
  internal_representation_ = std::shared_ptr<void>(
      internal_occupancy_map, internal_occupancy_map.get());
}

VoxelOccupancyMap::VoxelOccupancyMap(const VoxelOccupancyMap& other) {
  internal_representation_ = CopyInternalRepresentation(other);
}

VoxelOccupancyMap::VoxelOccupancyMap(VoxelOccupancyMap&& other) {
  internal_representation_ = std::move(other.internal_representation_);
  other.InitializeEmpty();
}

VoxelOccupancyMap& VoxelOccupancyMap::operator=(
    const VoxelOccupancyMap& other) {
  if (this != &other) {
    internal_representation_ = CopyInternalRepresentation(other);
  }
  return *this;
}

VoxelOccupancyMap& VoxelOccupancyMap::operator=(VoxelOccupancyMap&& other) {
  if (this != &other) {
    internal_representation_ = std::move(other.internal_representation_);
    other.InitializeEmpty();
  }
  return *this;
}

VoxelSignedDistanceField VoxelOccupancyMap::ExportSignedDistanceField(
    const VoxelSignedDistanceField::GenerationParameters& parameters) const {
  const auto& internal_occupancy_map = internal::GetInternalOccupancyMap(*this);

  auto internal_sdf = std::make_shared<SignedDistanceField<float>>(
      internal_occupancy_map.ExtractSignedDistanceFieldFloat(
          internal::ToVGT(parameters)));

  auto internal_sdf_representation =
      std::shared_ptr<void>(internal_sdf, internal_sdf.get());

  return VoxelSignedDistanceField(internal_sdf_representation);
}

const std::string& VoxelOccupancyMap::parent_body_name() const {
  const auto& internal_occupancy_map = internal::GetInternalOccupancyMap(*this);
  return internal_occupancy_map.GetFrame();
}

bool VoxelOccupancyMap::is_empty() const {
  const auto& internal_occupancy_map = internal::GetInternalOccupancyMap(*this);
  return !internal_occupancy_map.IsInitialized();
}

void VoxelOccupancyMap::InitializeEmpty() {
  auto internal_occupancy_map = std::make_shared<OccupancyMap>();
  internal_representation_ = std::shared_ptr<void>(
      internal_occupancy_map, internal_occupancy_map.get());
}

}  // namespace planning
}  // namespace drake
