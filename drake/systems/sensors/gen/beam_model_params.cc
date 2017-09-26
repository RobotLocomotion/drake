#include "drake/systems/sensors/gen/beam_model_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace systems {
namespace sensors {

const int BeamModelParamsIndices::kNumCoordinates;
const int BeamModelParamsIndices::kLambdaShort;
const int BeamModelParamsIndices::kSigmaHit;
const int BeamModelParamsIndices::kProbabilityShort;
const int BeamModelParamsIndices::kProbabilityMiss;
const int BeamModelParamsIndices::kProbabilityUniform;

const std::vector<std::string>& BeamModelParamsIndices::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "lambda_short", "sigma_hit", "probability_short", "probability_miss",
          "probability_uniform",
      });
  return coordinates.access();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
