#include "drake/planning/dev/voxel_signed_distance_field_internal.h"

#include <common_robotics_utilities/parallelism.hpp>

namespace drake {
namespace planning {
namespace internal {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using voxelized_geometry_tools::SignedDistanceField;
using voxelized_geometry_tools::SignedDistanceFieldGenerationParameters;

const SignedDistanceField<float>& GetInternalSignedDistanceField(
    const VoxelSignedDistanceField& distance_field) {
  return *reinterpret_cast<const SignedDistanceField<float>*>(
      distance_field.internal_representation());
}

SignedDistanceFieldGenerationParameters<float> ToVGT(
    const VoxelSignedDistanceField::GenerationParameters& parameters) {
  return SignedDistanceFieldGenerationParameters<float>(
      parameters.oob_value,
      DegreeOfParallelism(parameters.parallelism.num_threads()),
      parameters.unknown_is_filled, parameters.add_virtual_border);
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
