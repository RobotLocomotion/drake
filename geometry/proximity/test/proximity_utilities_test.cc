#include "drake/geometry/proximity/proximity_utilities.h"

#include <memory>
#include <sstream>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using fcl::CollisionGeometryd;
using fcl::CollisionObjectd;

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

// This tests writing to and extracting from an fcl object,
GTEST_TEST(EncodedData, ConstructionFromFclObject) {
  GeometryId id_A = GeometryId::get_new_id();
  EncodedData data_A(id_A, true);
  CollisionObjectd object(std::shared_ptr<CollisionGeometryd>(nullptr));

  data_A.write_to(&object);
  {
    EncodedData data_read(object);
    EXPECT_TRUE(data_read.is_dynamic());
    EXPECT_EQ(data_read.id(), data_A.id());
  }

  data_A.set_anchored();
  data_A.write_to(&object);
  {
    EncodedData data_read(object);
    EXPECT_FALSE(data_read.is_dynamic());
    EXPECT_EQ(data_read.id(), data_A.id());
  }
}

GTEST_TEST(ShapeName, SimpleReification) {
  ShapeName name;

  EXPECT_EQ(name.string(), "");

  Sphere(0.5).Reify(&name);
  EXPECT_EQ(name.string(), "Sphere");

  Cylinder(1, 2).Reify(&name);
  EXPECT_EQ(name.string(), "Cylinder");

  Box(1, 2, 3).Reify(&name);
  EXPECT_EQ(name.string(), "Box");

  HalfSpace().Reify(&name);
  EXPECT_EQ(name.string(), "HalfSpace");

  Mesh("filepath", 1.0).Reify(&name);
  EXPECT_EQ(name.string(), "Mesh");

  Convex("filepath", 1.0).Reify(&name);
  EXPECT_EQ(name.string(), "Convex");
}

GTEST_TEST(ShapeName, ReifyOnConstruction) {
  EXPECT_EQ(ShapeName(Sphere(0.5)).string(), "Sphere");
  EXPECT_EQ(ShapeName(Cylinder(1, 2)).string(), "Cylinder");
  EXPECT_EQ(ShapeName(Box(1, 2, 3)).string(), "Box");
  EXPECT_EQ(ShapeName(HalfSpace()).string(), "HalfSpace");
  EXPECT_EQ(ShapeName(Mesh("filepath", 1.0)).string(), "Mesh");
  EXPECT_EQ(ShapeName(Convex("filepath", 1.0)).string(), "Convex");
}

GTEST_TEST(ShapeName, Stremaing) {
  ShapeName name(Sphere(0.5));
  std::stringstream ss;
  ss << name;
  EXPECT_EQ(name.string(), "Sphere");
  EXPECT_EQ(ss.str(), name.string());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
