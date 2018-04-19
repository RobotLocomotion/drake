#include "drake/tools/vector_gen/test/gen/sample.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace drake {
namespace tools {
namespace test {

const int SampleIndices::kNumCoordinates;
const int SampleIndices::kX;
const int SampleIndices::kTwoWord;
const int SampleIndices::kAbsone;

const std::vector<std::string>& SampleIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "x",         // BR
          "two_word",  // BR
          "absone",    // BR
      });
  return coordinates.access();
}

}  // namespace test
}  // namespace tools
}  // namespace drake
