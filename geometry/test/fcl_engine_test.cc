#include "drake/geometry/fcl_engine.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {

class FclEngineTester {
 public:
  FclEngineTester() = delete;

  template <typename T>
  static const fcl::CollisionObjectd& GetDynamicCollisionObject(
      int index, const FclEngine<T>& engine) {
    return *engine.dynamic_objects_.at(index).get();
  }

  template <typename T>
  static const fcl::CollisionObjectd& GetAnchoredCollisionObject(
      int index, const FclEngine<T>& engine) {
    return *engine.anchored_objects_.at(index).get();
  }
};

namespace {

using Eigen::Translation3d;
using std::move;

// Test simple addition of dynamic geometry.
GTEST_TEST(FclEngineTests, AddDynamicGeometry) {
  FclEngine<double> engine;
  Sphere sphere{0.5};
  GeometryIndex index = engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(index, 0);
  EXPECT_EQ(engine.num_geometries(), 1);
  EXPECT_EQ(engine.num_anchored(), 0);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// Tests simple addition of anchored geometry.
GTEST_TEST(FclEngineTests, AddAchoredGeometry) {
  FclEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex index = engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(index, 0);
  EXPECT_EQ(engine.num_geometries(), 1);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 0);
}

// Tests addition of both dynamic and anchored geometry.
GTEST_TEST(FclEngineTests, AddMixedGeometry) {
  FclEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex a_index = engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(a_index, 0);
  GeometryIndex g_index = engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(g_index, 0);
  EXPECT_EQ(engine.num_geometries(), 2);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

// Tests the copy semantics of the FclEngine -- the copy is a complete, deep
// copy.
GTEST_TEST(FclEngineTests, CopySemantics) {
  FclEngine<double> ref_engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex a_index = ref_engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(a_index, 0);
  GeometryIndex g_index = ref_engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(g_index, 0);

  // Function for validating that two objects are different objects with
  // "identical" data. The test isn't exhaustive (for example, the parameters
  // of the particular geometric shape are not compared--instead, we compare the
  // AABBs).
  auto ValidateObject = [](const fcl::CollisionObject<double>& test,
                           const fcl::CollisionObject<double>& ref) {
    // NOTE: The user data is not an actual pointer that would change due
    // to a deep copy. It is an encoded index that should be the same in a
    // copy.
    EXPECT_EQ(test.getUserData(), ref.getUserData());
    EXPECT_NE(&test, &ref);
    EXPECT_EQ(test.getNodeType(), ref.getNodeType());
    EXPECT_EQ(test.getObjectType(), ref.getObjectType());
    EXPECT_EQ(test.getAABB().center(), ref.getAABB().center());
    EXPECT_EQ(test.getAABB().width(), ref.getAABB().width());
    EXPECT_EQ(test.getAABB().height(), ref.getAABB().height());
    EXPECT_EQ(test.getAABB().depth(), ref.getAABB().depth());
  };

  // Function for validating that the test engine is a deep copy of the
  // reference engine.
  auto ValidateCopy = [&ref_engine,
                       ValidateObject](const FclEngine<double>& test_engine) {
    EXPECT_EQ(test_engine.num_geometries(), ref_engine.num_geometries());
    EXPECT_EQ(test_engine.num_anchored(), ref_engine.num_anchored());
    EXPECT_EQ(test_engine.num_dynamic(), ref_engine.num_dynamic());

    for (int i = 0; i < test_engine.num_dynamic(); ++i) {
      const fcl::CollisionObjectd& test_object =
          FclEngineTester::GetDynamicCollisionObject(i, test_engine);
      const fcl::CollisionObjectd& ref_object =
          FclEngineTester::GetDynamicCollisionObject(i, ref_engine);
      ValidateObject(test_object, ref_object);
    }

    for (int i = 0; i < test_engine.num_anchored(); ++i) {
      const fcl::CollisionObjectd& test_object =
          FclEngineTester::GetAnchoredCollisionObject(i, test_engine);
      const fcl::CollisionObjectd& ref_object =
          FclEngineTester::GetAnchoredCollisionObject(i, ref_engine);
      ValidateObject(test_object, ref_object);
    }
  };

  FclEngine<double> copy_construct(ref_engine);
  ValidateCopy(copy_construct);

  FclEngine<double> copy_assign;
  copy_assign = ref_engine;
  ValidateCopy(copy_assign);
}

// Tests the move semantics of the FclEngine -- the copy is a complete, deep
// copy.
GTEST_TEST(FclEngineTests, MoveSemantics) {
  // TODO(SeanCurtis-TRI): Update this when the move semantics are no longer
  // simply a copy.
  FclEngine<double> engine;
  Sphere sphere{0.5};
  Isometry3<double> pose = Isometry3<double>::Identity();
  AnchoredGeometryIndex a_index = engine.AddAnchoredGeometry(sphere, pose);
  EXPECT_EQ(a_index, 0);
  GeometryIndex g_index = engine.AddDynamicGeometry(sphere);
  EXPECT_EQ(g_index, 0);

  FclEngine<double> move_construct(move(engine));
  EXPECT_EQ(move_construct.num_geometries(), 2);
  EXPECT_EQ(move_construct.num_anchored(), 1);
  EXPECT_EQ(move_construct.num_dynamic(), 1);
  // TODO(SeanCurtis-TRI): This confirms the temporary copy-like behavior of
  // the move semantics. Change it when it is a legitimate move.
  EXPECT_EQ(engine.num_geometries(), 2);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 1);

  FclEngine<double> move_assign;
  move_assign = move(engine);
  EXPECT_EQ(move_assign.num_geometries(), 2);
  EXPECT_EQ(move_assign.num_anchored(), 1);
  EXPECT_EQ(move_assign.num_dynamic(), 1);
  // TODO(SeanCurtis-TRI): This confirms the temporary copy-like behavior of
  // the move semantics.;
  EXPECT_EQ(engine.num_geometries(), 2);
  EXPECT_EQ(engine.num_anchored(), 1);
  EXPECT_EQ(engine.num_dynamic(), 1);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
