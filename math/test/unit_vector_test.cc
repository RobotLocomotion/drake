#include "drake/math/unit_vector.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace math {
namespace {

GTEST_TEST(UnitVectorTest, ThrowOrWarnIfNotUnitVector) {
  double vector_squared_magnitude;

  // Verify that no exception is thrown for a valid unit vector.
  const Vector3<double> unit_vector(1.0, 0.0, 0.0);
  DRAKE_EXPECT_NO_THROW(vector_squared_magnitude =
      ThrowUnlessVectorIsMagnitudeOne(unit_vector, "UnusedFunctionName"));
  EXPECT_EQ(vector_squared_magnitude, unit_vector.squaredNorm());

  // Verify that an exception is thrown for an invalid unit vector.
  const Vector3<double> not_unit_vector(1.0, 2.0, 3.0);
  std::string expected_message =
      "SomeFunctionName\\(\\): The unit_vector argument 1 2 3 is not a unit"
      " vector\\.";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ThrowUnlessVectorIsMagnitudeOne(not_unit_vector, "SomeFunctionName"),
      expected_message);

  // No message should be written to the log file for a valid unit vector.
  vector_squared_magnitude =
      WarnUnlessVectorIsMagnitudeOne(unit_vector, "UnusedFunctionName");
  EXPECT_EQ(vector_squared_magnitude, unit_vector.squaredNorm());

  // A message should be written to the log file for an invalid unit vector.
  vector_squared_magnitude =
      WarnUnlessVectorIsMagnitudeOne(not_unit_vector, "SomeFunctionName");
  EXPECT_EQ(vector_squared_magnitude, not_unit_vector.squaredNorm());
}

}  // namespace
}  // namespace math
}  // namespace drake
