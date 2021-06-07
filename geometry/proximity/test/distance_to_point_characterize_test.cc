#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/test/characterization_utilities.h"
#include "drake/geometry/query_results/signed_distance_to_point.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

/* @file This provides the test that supports the values contained in the
 table documented for QueryObject::ComputeSignedDistanceToPoint in
 query_object.h. */

namespace drake {
namespace geometry {
namespace internal {
namespace point_distance {
namespace {

using std::vector;

/* Implementation of DistanceCallback for signed distance. */
template <typename T>
class PointDistanceCallback : public DistanceCallback<T> {
 public:
  bool Invoke(fcl::CollisionObjectd* obj_A, fcl::CollisionObjectd* obj_B,
              const CollisionFilter*,
              const std::unordered_map<GeometryId, math::RigidTransform<T>>*
                  X_WGs) override {
    // We treat the first sphere as representing the query point.
    DRAKE_DEMAND(obj_A->collisionGeometry()->getNodeType() == fcl::GEOM_SPHERE);
    const GeometryId point_id = EncodedData(*obj_A).id();
    const Vector3<T> p_WQ = X_WGs->at(point_id).translation();
    CallbackData<T> data(obj_A, std::numeric_limits<double>::infinity(), p_WQ,
                         X_WGs, &results_);
    double max_distance = std::numeric_limits<double>::infinity();
    return Callback<T>(obj_A, obj_B, &data, max_distance);
  }

  void ClearResults() override { results_.clear(); }

  int GetNumResults() const override {
    return static_cast<int>(results_.size());
  }

  T GetFirstSignedDistance() const override { return results_[0].distance; }

 private:
  vector<SignedDistanceToPoint<T>> results_;
};

template <typename T>
class CharacterizePointDistanceResultTest : public CharacterizeResultTest<T> {
 public:
  CharacterizePointDistanceResultTest()
      : CharacterizeResultTest<T>(
            std::make_unique<PointDistanceCallback<T>>()) {}

  vector<double> TestDistances() const final {
    return {-this->kDistance, this->kDistance};
  }

  /* Although we're representing the point as a sphere to fit into the
   framework, we can't allow the default configurations to be generated because
   the zero-radius sphere will fail the tests that seek to confirm it has
   sufficient measure to support penetration. So, we'll create our own
   configurations with a single arbitrary tangent plane to the point, and the
   default sample tangent planes for the other shape.  */
  vector<Configuration<T>> MakeConfigurations(
      const Shape& shape_A, const Shape& shape_B,
      const vector<double>& signed_distances) const override {
    // For this test, we require shape A to be a zero-radius sphere.
    DRAKE_DEMAND(ShapeName(shape_A).name() == "Sphere");
    vector<Configuration<T>> configs;
    // We'll create a tangent plane to the point with an arbitrary normal
    // direction.
    const Vector3<T> p_WA{0, 0, 0};
    const Vector3<T> a_norm_W = Vector3<T>{1, -2, 0.5}.normalized();
    for (const double signed_distance : signed_distances) {
      const Vector3<T> p_WC = p_WA + a_norm_W * signed_distance;
      for (const auto& sample_B :
           ShapeConfigurations<T>(shape_B, signed_distance).configs()) {
        /* If the sample point is not going to be the nearest point to the
         query point, skip it. */
        if (sample_B.max_depth < -signed_distance) continue;
        const Vector3<T>& p_WB = sample_B.point;
        const Vector3<T>& b_norm_W = sample_B.normal;
        const std::string relates =
            signed_distance < 0
                ? " inside near "
                : (signed_distance > 0 ? " outside near " : " touching at ");
        configs.push_back(
            {AlignPlanes(p_WC, a_norm_W, p_WB, b_norm_W), signed_distance,
             "point" + relates + sample_B.description});
      }
    }
    return configs;
  }
};

/* *-Mesh has not been implemented because Mesh is represented by Convex.
 However, this single test will detect when that condition is no longer true
 and call for implementation of *-Mesh tests. */
GTEST_TEST(CharacterizePointDistanceResultTest, MeshMesh) {
  ASSERT_TRUE(MeshIsConvex());
}

class DoubleTest : public CharacterizePointDistanceResultTest<double>,
                   public testing::WithParamInterface<QueryInstance> {};

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    PointDistance, DoubleTest,
    testing::Values(
        QueryInstance(kPoint, kBox, 2e-15),
        QueryInstance(kPoint, kCapsule, 4e-15),
        QueryInstance(kPoint, kConvex, kIgnores),
        QueryInstance(kPoint, kCylinder, 3e-15),
        QueryInstance(kPoint, kEllipsoid, 3e-5),
        QueryInstance(kPoint, kHalfSpace, 5e-15),
        QueryInstance(kPoint, kSphere, 4e-15)),
    QueryInstanceName);
// clang-format on

TEST_P(DoubleTest, Characterize) {
  this->RunCharacterization(GetParam(), false /* is_symmetric */);
}

class AutoDiffTest : public CharacterizePointDistanceResultTest<AutoDiffXd>,
                     public testing::WithParamInterface<QueryInstance> {};

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    PointDistance, AutoDiffTest,
    testing::Values(
        QueryInstance(kPoint, kBox, 1e-15),
        QueryInstance(kPoint, kCapsule, 4e-15),
        QueryInstance(kPoint, kConvex, kIgnores),
        QueryInstance(kPoint, kCylinder, kIgnores),
        QueryInstance(kPoint, kEllipsoid, kIgnores),
        QueryInstance(kPoint, kHalfSpace, 5e-15),
        QueryInstance(kPoint, kSphere, 3e-15)),
    QueryInstanceName);
// clang-format on

TEST_P(AutoDiffTest, Characterize) {
  this->RunCharacterization(GetParam(), false /* is_symmetric */);
}

}  // namespace
}  // namespace point_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
