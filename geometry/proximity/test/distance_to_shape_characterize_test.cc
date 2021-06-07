#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/distance_to_shape_callback.h"
#include "drake/geometry/proximity/test/characterization_utilities.h"
#include "drake/math/rigid_transform.h"

/* @file This provides the test that supports the values contained in the
 table documented for QueryObject::ComputeSignedDistancePairwiseClosestPoints in
 query_object.h. */

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {
namespace {

using std::make_unique;
using std::vector;

/* Implementation of DistanceCallback for signed distance. */
template <typename T>
class SignedDistanceCallback : public DistanceCallback<T> {
 public:
  bool Invoke(fcl::CollisionObjectd* obj_A, fcl::CollisionObjectd* obj_B,
              const CollisionFilter* collision_filter,
              const std::unordered_map<GeometryId, math::RigidTransform<T>>*
                  X_WGs) override {
    CallbackData<T> data(collision_filter, X_WGs,
                         std::numeric_limits<double>::infinity(), &results_);
    data.request.enable_signed_distance = true;
    data.request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    /* TODO(#14731) This reflects the fact that ProximityEngine hard
     codes this value to 1e-6. However, the results of multiple characterization
     tests depend on this value. Specifically:

       Box-Capsule, Box-Cylinder, Box-Ellipsoid, Capsule-Capsule,
       Capsule-Convex, Capsule-Cylinder, Capsule-Ellipsoid, Convex-Cylinder,
       Convex-Ellipsoid, Convex-Sphere, Cylinder-Ellipsoid, Ellipsoid-Ellipsoid,
       and Ellipsoid-Sphere

      If/when that parameter is exposed in the public API, the table should be
      updated to reflect the results that depend on that parameter and the test
      here should be expressed relative to this quantity in support. */
    data.request.distance_tolerance = 1e-6;
    /* We're not testing the logic for limiting results based on a maximum
     distance. So, we'll simply set it to infinity. */
    double max_distance = std::numeric_limits<double>::infinity();
    return Callback<T>(obj_A, obj_B, &data, max_distance);
  }

  void ClearResults() override { results_.clear(); }

  int GetNumResults() const override {
    return static_cast<int>(results_.size());
  }

  T GetFirstSignedDistance() const override { return results_[0].distance; }

 private:
  vector<SignedDistancePair<T>> results_;
};

template <typename T>
class CharacterizeShapeDistanceResultTest : public CharacterizeResultTest<T> {
 public:
  CharacterizeShapeDistanceResultTest()
      : CharacterizeResultTest<T>(make_unique<SignedDistanceCallback<T>>()) {}

  std::vector<double> TestDistances() const final {
    return {-this->kDistance, this->kDistance};
  }
};

/* *-Mesh has not been implemented because Mesh is represented by Convex.
 However, this single test will detect when that condition is no longer true
 and call for implementation of *-Mesh tests. */
GTEST_TEST(CharacterizeShapeDistanceResultTest, MeshMesh) {
  ASSERT_TRUE(MeshIsConvex());
}

class DoubleTest : public CharacterizeShapeDistanceResultTest<double>,
                   public testing::WithParamInterface<QueryInstance> {};

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    ShapeDistance, DoubleTest,
    testing::Values(
        QueryInstance(kBox, kBox, 4e-15),
        QueryInstance(kBox, kCapsule, 3e-6),
        QueryInstance(kBox, kConvex, 3e-15),
        QueryInstance(kBox, kCylinder, 6e-6),
        QueryInstance(kBox, kEllipsoid, 9e-6),
        QueryInstance(kBox, kHalfSpace, kThrows),
        QueryInstance(kBox, kSphere, 3e-15),

        QueryInstance(kCapsule, kCapsule, 2e-5),
        QueryInstance(kCapsule, kConvex, 2e-5),
        QueryInstance(kCapsule, kCylinder, 1e-5),
        QueryInstance(kCapsule, kEllipsoid, 5e-6),
        QueryInstance(kCapsule, kHalfSpace, kThrows),
        QueryInstance(kCapsule, kSphere, 6e-15),

        QueryInstance(kConvex, kConvex, 3e-15),
        QueryInstance(kConvex, kCylinder, 6e-6),
        QueryInstance(kConvex, kEllipsoid, 9e-6),
        QueryInstance(kConvex, kHalfSpace, kThrows),
        QueryInstance(kConvex, kSphere, 3e-6),

        QueryInstance(kCylinder, kCylinder, 2e-5),
        QueryInstance(kCylinder, kEllipsoid, 5e-5),
        QueryInstance(kCylinder, kHalfSpace, kThrows),
        QueryInstance(kCylinder, kSphere, 5e-15),

        QueryInstance(kEllipsoid, kEllipsoid, 2e-5),
        QueryInstance(kEllipsoid, kHalfSpace, kThrows),
        QueryInstance(kEllipsoid, kSphere, 4e-5),

        QueryInstance(kHalfSpace, kHalfSpace, kThrows),
        QueryInstance(kHalfSpace, kSphere, 3e-15),

        QueryInstance(kSphere, kSphere, 6e-15)),
    QueryInstanceName);
// clang-format on

TEST_P(DoubleTest, Characterize) {
  this->RunCharacterization(GetParam());
}

class AutoDiffTest : public CharacterizeShapeDistanceResultTest<AutoDiffXd>,
                     public testing::WithParamInterface<QueryInstance> {};

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    ShapeDistance, AutoDiffTest,
    testing::Values(
        QueryInstance(kBox, kBox, kThrows),
        QueryInstance(kBox, kCapsule, kThrows),
        QueryInstance(kBox, kConvex, kThrows),
        QueryInstance(kBox, kCylinder, kThrows),
        QueryInstance(kBox, kEllipsoid, kThrows),
        QueryInstance(kBox, kHalfSpace, kThrows),
        QueryInstance(kBox, kSphere, 2e-15),

        QueryInstance(kCapsule, kCapsule, kThrows),
        QueryInstance(kCapsule, kConvex, kThrows),
        QueryInstance(kCapsule, kCylinder, kThrows),
        QueryInstance(kCapsule, kEllipsoid, kThrows),
        QueryInstance(kCapsule, kHalfSpace, kThrows),
        QueryInstance(kCapsule, kSphere, kThrows),

        QueryInstance(kConvex, kConvex, kThrows),
        QueryInstance(kConvex, kCylinder, kThrows),
        QueryInstance(kConvex, kEllipsoid, kThrows),
        QueryInstance(kConvex, kHalfSpace, kThrows),
        QueryInstance(kConvex, kSphere, kThrows),

        QueryInstance(kCylinder, kCylinder, kThrows),
        QueryInstance(kCylinder, kEllipsoid, kThrows),
        QueryInstance(kCylinder, kHalfSpace, kThrows),
        QueryInstance(kCylinder, kSphere, kThrows),

        QueryInstance(kEllipsoid, kEllipsoid, kThrows),
        QueryInstance(kEllipsoid, kHalfSpace, kThrows),
        QueryInstance(kEllipsoid, kSphere, kThrows),

        QueryInstance(kHalfSpace, kHalfSpace, kThrows),
        QueryInstance(kHalfSpace, kSphere, 2e-15),

        QueryInstance(kSphere, kSphere, 5e-15)),
    QueryInstanceName);
// clang-format on

TEST_P(AutoDiffTest, Characterize) {
  this->RunCharacterization(GetParam());
}

}  // namespace
}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
