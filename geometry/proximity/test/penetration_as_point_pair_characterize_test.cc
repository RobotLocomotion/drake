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

  std::vector<double> TestDistances() const final {
    return {-this->kDistance};
  }
};

class DoubleTest : public CharacterizePointPairResultTest<double>,
                   public testing::WithParamInterface<QueryInstance> {};

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    PenetrationAsPointPair, DoubleTest,
    testing::Values(
        QueryInstance(kCapsule, kCapsule, 2e-5),
        QueryInstance(kCapsule, kEllipsoid, 2e-4),
        QueryInstance(kCapsule, kSphere, 3e-15),


        QueryInstance(kEllipsoid, kEllipsoid, 5e-4),
        QueryInstance(kEllipsoid, kSphere, 2e-4),


        QueryInstance(kSphere, kSphere, 3e-15)),
    QueryInstanceName);
// clang-format on

TEST_P(DoubleTest, Characterize) {
  this->RunCharacterization(GetParam());
}

class AutoDiffTest : public CharacterizePointPairResultTest<AutoDiffXd>,
                     public testing::WithParamInterface<QueryInstance> {};

// clang-format off
INSTANTIATE_TEST_SUITE_P(
    PenetrationAsPointPair, AutoDiffTest,
    testing::Values(
        QueryInstance(kCapsule, kCapsule, kThrows),
        QueryInstance(kCapsule, kEllipsoid, kThrows),
        QueryInstance(kCapsule, kSphere, 3e-15),

        QueryInstance(kEllipsoid, kEllipsoid, kThrows),
        QueryInstance(kEllipsoid, kSphere, kThrows),


        QueryInstance(kSphere, kSphere, 3e-15)),
    QueryInstanceName);
// clang-format on

TEST_P(AutoDiffTest, Characterize) {
  this->RunCharacterization(GetParam());
}

}  // namespace
}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
