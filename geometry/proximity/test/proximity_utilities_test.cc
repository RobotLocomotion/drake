#include "drake/geometry/proximity/proximity_utilities.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {

// Construct a couple of encodings and test their properties.
GTEST_TEST(EncodedData, ConstructorAndProperties) {
  GeometryId id_A = GeometryId::get_new_id();
  EncodedData data_A(id_A, true);
  EXPECT_EQ(data_A.id(), id_A);
  EXPECT_TRUE(data_A.is_dynamic());


  GeometryId id_B = GeometryId::get_new_id();
  EncodedData data_B(id_B, false);
  EXPECT_EQ(data_B.id(), id_B);
  EXPECT_FALSE(data_B.is_dynamic());
}

GTEST_TEST(EncodedData, FactoryConstruction) {
  GeometryId id_A = GeometryId::get_new_id();

  EncodedData dynamic = EncodedData::encode_dynamic(id_A);
  EXPECT_EQ(dynamic.id(), id_A);
  EXPECT_TRUE(dynamic.is_dynamic());

  EncodedData anchored = EncodedData::encode_anchored(id_A);
  EXPECT_EQ(anchored.id(), id_A);
  EXPECT_FALSE(anchored.is_dynamic());
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
