#pragma once

#include <common_robotics_utilities/parallelism.hpp>
#include <voxelized_geometry_tools/signed_distance_field.hpp>

#include "drake/planning/experimental/voxel_signed_distance_field.h"

namespace drake {
namespace planning {
namespace experimental {
namespace internal {
// Retrieve the internal SignedDistanceField<float> from a
// VoxelSignedDistanceField instance.
inline const voxelized_geometry_tools::SignedDistanceField<float>&
GetInternalSignedDistanceField(const VoxelSignedDistanceField& distance_field) {
  using voxelized_geometry_tools::SignedDistanceField;
  return *reinterpret_cast<const SignedDistanceField<float>*>(
      distance_field.internal_representation());
}

// Convert the provided `parameters` into the paramter class used by
// voxelized_geometry_tools for signed distance field generation.
inline voxelized_geometry_tools::SignedDistanceFieldGenerationParameters<float>
ToVGT(const VoxelSignedDistanceField::GenerationParameters& parameters) {
  using common_robotics_utilities::parallelism::DegreeOfParallelism;
  using voxelized_geometry_tools::SignedDistanceFieldGenerationParameters;
  return SignedDistanceFieldGenerationParameters<float>(
      parameters.oob_value,
      DegreeOfParallelism(parameters.parallelism.num_threads()),
      parameters.unknown_is_filled, parameters.add_virtual_border);
}

}  // namespace internal
}  // namespace experimental
}  // namespace planning
}  // namespace drake
