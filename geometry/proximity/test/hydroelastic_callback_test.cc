#include "drake/geometry/proximity/hydroelastic_callback.h"

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {
namespace {

using Eigen::Vector3d;
using fcl::Boxd;
using fcl::CollisionObjectd;
using fcl::Halfspaced;
using fcl::Sphered;
using math::RigidTransform;
using std::make_shared;
using std::make_unique;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

// TODO(SeanCurtis-TRI): When autodiff is more generally supported, replace the
//  simple exception-expecting call (AutoDiffBlanketFailure) and add AutoDiff
//  into this type list: ScalarTypes.

// Confirmation of the short-term behavior that invoking the callback on
// potentially colliding geometry using AutoDiffXd-valued transforms will throw.
// Configure a scenario that would otherwise pass with double-valued poses.
GTEST_TEST(HydroelasticCallbackAutodiff, AutoDiffBlanketFailure) {
  // Note: this code should duplicate the valid double-valued configuration in
  // the ValidPairProducesResult test. Its only point of failure should be that
  // it is AutoDiffXd valued.
  const double radius = 0.25;
  const double cube_size = 0.4;
  GeometryId id_A = GeometryId::get_new_id();
  GeometryId id_B = GeometryId::get_new_id();
  EncodedData data_A(id_A, true);
  EncodedData data_B(id_B, true);
  CollisionFilterLegacy collision_filter;
  collision_filter.AddGeometry(data_A.encoding());
  collision_filter.AddGeometry(data_B.encoding());
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs{
      {id_A, RigidTransform<AutoDiffXd>::Identity()},
      {id_B, RigidTransform<AutoDiffXd>(
                 Vector3<AutoDiffXd>{0, 0, 0.9 * (radius + cube_size / 2)})}};

  CollisionObjectd object_A(make_shared<Sphered>(radius));
  data_A.write_to(&object_A);
  CollisionObjectd object_B(make_shared<Boxd>(cube_size, cube_size, cube_size));
  data_B.write_to(&object_B);

  vector<ContactSurface<AutoDiffXd>> surfaces;
  CallbackData<AutoDiffXd> data(&collision_filter, &X_WGs, &surfaces);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Callback<AutoDiffXd>(&object_A, &object_B, &data), std::logic_error,
      "AutoDiff-valued ContactSurface calculation between meshes is not"
      "currently supported");
}

// Infrastructure to repeat tests on both double and AutoDiffXd.
using ScalarTypes = ::testing::Types<double>;
TYPED_TEST_CASE(HydroelasticCallbackTyped, ScalarTypes);

template <typename T>
class HydroelasticCallbackTyped : public ::testing::Test {
 protected:
  void SetUp() override {
    // Configures a world with a sphere slightly overlapping a box.
    id_sphere_ = GeometryId::get_new_id();
    id_box_ = GeometryId::get_new_id();
    EncodedData data_A(id_sphere_, true);
    EncodedData data_B(id_box_, true);
    collision_filter_.AddGeometry(data_A.encoding());
    collision_filter_.AddGeometry(data_B.encoding());
    // Leave box centered on origin.
    X_WGs_.insert({id_box_, RigidTransform<T>::Identity()});
    // Position sphere so that it is just far enough from the origin that the
    // bottom of the sphere intersects the top of the box.
    X_WGs_.insert({id_sphere_, RigidTransform<T>(Vector3<T>{
                                   0, 0, 0.9 * (radius_ + cube_size_ / 2)})});

    sphere_ = make_unique<CollisionObjectd>(make_shared<Sphered>(radius_));
    data_A.write_to(sphere_.get());
    box_ = make_unique<CollisionObjectd>(
        make_shared<Boxd>(cube_size_, cube_size_, cube_size_));
    data_B.write_to(box_.get());
  }

  CollisionFilterLegacy collision_filter_;
  unordered_map<GeometryId, RigidTransform<T>> X_WGs_;
  GeometryId id_sphere_{};
  GeometryId id_box_{};
  const double radius_{0.25};
  const double cube_size_{0.4};
  unique_ptr<CollisionObjectd> sphere_;
  unique_ptr<CollisionObjectd> box_;
};

// TODO(SeanCurtis-TRI): This test relies on the hackiness of the code when it
//  it was introduced -- I.e., the *only* support that exists is for a soft
//  sphere and rigid box. Update/replace this test when the geneeral support
//  for evaluating hydroelastic contact is introduced.

// Confirms that a colliding collision pair (with supported hydroelastic
// representations) produces a result. This doesn't test the actual data -- it
// assumes the function responsible for computing that result has been
// successfully tested.
TYPED_TEST(HydroelasticCallbackTyped, ValidPairProducesResult) {
  using T = TypeParam;

  vector<ContactSurface<T>> surfaces;
  CallbackData<T> data(&this->collision_filter_, &this->X_WGs_, &surfaces);
  EXPECT_NO_THROW(
      Callback<T>(this->sphere_.get(), this->box_.get(), &data));
  EXPECT_EQ(surfaces.size(), 1u);
}

}  // namespace
}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
