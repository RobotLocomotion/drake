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

#define CHARACTERIZE_TEST(shape1, shape2, expect, precision)                   \
TYPED_TEST(CharacterizePointPairResultTest, shape1##shape2) {                  \
  this->RunCharacterization(expect<TypeParam>(precision), this->shape1(),      \
                            this->shape2(), vector<double>{-this->kDistance}); \
}

#define SELF_CHARACTERIZE_TEST(shape, expect, precision)                       \
TYPED_TEST(CharacterizePointPairResultTest, shape##shape) {                    \
  this->RunCharacterization(expect<TypeParam>(precision), this->shape(),       \
                            this->shape(true),                                 \
                            vector<double>{-this->kDistance});                 \
}

SELF_CHARACTERIZE_TEST(capsule, ExpectDoubleOnly, 2e-5)
CHARACTERIZE_TEST(capsule, ellipsoid, ExpectDoubleOnly, 2e-4)
CHARACTERIZE_TEST(capsule, sphere, ExpectAllT, 3e-15)

SELF_CHARACTERIZE_TEST(ellipsoid, ExpectDoubleOnly, 5e-4)
CHARACTERIZE_TEST(ellipsoid, sphere, ExpectDoubleOnly, 2e-4)

SELF_CHARACTERIZE_TEST(sphere, ExpectAllT, 3e-15)

#undef CHARACTERIZE_TEST
#undef SELF_CHARACTERIZE_TEST
}  // namespace
}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
