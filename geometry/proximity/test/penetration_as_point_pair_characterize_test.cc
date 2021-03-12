#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"
#include "drake/geometry/proximity/test/characterization_utilities.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

/* @file This provides the test that supports the values contained in the
 table documented for QueryObject::ComputeSignedDistancePairwiseClosestPoints in
 query_object.h. */

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair {
namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RotationMatrix;
using std::make_unique;
using std::vector;

/* Implementation of DistanceCallback for penetration as point pair. */
template <typename T>
class PenetrationCallback : public DistanceCallback<T> {
 public:
  bool Invoke(fcl::CollisionObjectd* obj_A, fcl::CollisionObjectd* obj_B,
              const CollisionFilterLegacy* collision_filter,
              const std::unordered_map<GeometryId, math::RigidTransform<T>>*
                  X_WGs) override {
    CallbackData<T> data(collision_filter, X_WGs, &results_);
    return Callback<T>(obj_A, obj_B, &data);
  }

  void ClearResults() override { results_.clear(); }

  int GetNumResults() const override {
    return static_cast<int>(results_.size());
  }

  T GetFirstSignedDistance() const override { return -results_[0].depth; }

 private:
  vector<PenetrationAsPointPair<T>> results_;
};

template <typename T>
class CharacterizePointPairResultTest : public CharacterizeResultTest<T> {
 public:
  CharacterizePointPairResultTest()
      : CharacterizeResultTest<T>(make_unique<PenetrationCallback<T>>()) {}
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(CharacterizePointPairResultTest, ScalarTypes);

TYPED_TEST(CharacterizePointPairResultTest, SphereSphere) {
  using T = TypeParam;
  Expectation expect{.can_compute = true, .max_error = -1, .error_message = ""};
  if constexpr (std::is_same<T, double>::value) {
    expect.max_error = 8e-16;
  } else {
    expect.max_error = 3e-15;
  }
  // Orient the sphere arbitrarily and separate them *almost* by their combined
  // radii.
  const Sphere sphere = this->sphere();
  const RigidTransform<T> X_AB(
      RotationMatrix<T>(
          AngleAxis<T>(M_PI * 9 / 7, Vector3<T>{-1, 2, 1}.normalized())),
      Vector3<T>{1, -1, 2}.normalized() *
          (sphere.radius() * 2 - this->kDistance));
  vector<Configuration<T>> configs{{X_AB, -this->kDistance}};
  this->RunCharacterization(expect, sphere, sphere, configs);
}

}  // namespace
}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
