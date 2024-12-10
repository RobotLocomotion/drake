#pragma once

#include <voxelized_geometry_tools/signed_distance_field.hpp>

#include "drake/planning/dev/voxel_signed_distance_field.h"

namespace drake {
namespace planning {
namespace internal {
// Retrieve the internal SignedDistanceField<float> from a
// VoxelSignedDistanceField instance.
const voxelized_geometry_tools::SignedDistanceField<float>&
GetInternalSignedDistanceField(const VoxelSignedDistanceField& distance_field);

// Convert the provided `parameters` into the paramter class used by
// voxelized_geometry_tools for signed distance field generation.
voxelized_geometry_tools::SignedDistanceFieldGenerationParameters<float> ToVGT(
    const VoxelSignedDistanceField::GenerationParameters& parameters);

}  // namespace internal
}  // namespace planning
}  // namespace drake
