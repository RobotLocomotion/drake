#include "drake/geometry/proximity/hydroelastic_callback.h"

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {
namespace {

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

// Creates proximity properties sufficient to create a rigid representation
// for supported geometries.
ProximityProperties rigid_properties() {
  ProximityProperties props;
  const double chararacteristic_length = 0.1;
  AddRigidHydroelasticProperties(chararacteristic_length, &props);
  return props;
}

// Creates proximity properties sufficient to create a soft representation
// for supported geometries.
ProximityProperties soft_properties() {
  ProximityProperties props;
  AddContactMaterial(1e8, {}, {}, &props);
  const double chararacteristic_length = 0.25;
  AddSoftHydroelasticProperties(chararacteristic_length, &props);
  return props;
}

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

  hydroelastic::Geometries hydroelastic_geometries;
  hydroelastic_geometries.MaybeAddGeometry(Sphere(radius), id_A,
                                           soft_properties());
  hydroelastic_geometries.MaybeAddGeometry(Box(cube_size, cube_size, cube_size),
                                           id_B, rigid_properties());

  vector<ContactSurface<AutoDiffXd>> surfaces;
  CallbackData<AutoDiffXd> data(&collision_filter, &X_WGs,
                                &hydroelastic_geometries, &surfaces);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Callback<AutoDiffXd>(&object_A, &object_B, &data), std::logic_error,
      "AutoDiff-valued ContactSurface calculation between meshes is not"
      "currently supported");
}

// Infrastructure to repeat tests on both double and AutoDiffXd.
using ScalarTypes = ::testing::Types<double>;
TYPED_TEST_SUITE(HydroelasticCallbackTyped, ScalarTypes);

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

// Confirms that if the intersecting pair is missing hydroelastic representation
// that an exception is thrown. This test should apply *no* collision filters
// to guarantee that the body of the callback gets exercised in all cases.
TYPED_TEST(HydroelasticCallbackTyped,
           ThrowForMissingHydroelasticRepresentation) {
  using T = TypeParam;

  vector<ContactSurface<T>> surfaces;

  const Sphere sphere(this->radius_);
  const Box box = Box::MakeCube(this->cube_size_);

  // Case: neither geometry has representation.
  {
    Geometries hydroelastic_geometries;
    CallbackData<T> data(&this->collision_filter_, &this->X_WGs_,
                         &hydroelastic_geometries, &surfaces);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Callback<T>(this->sphere_.get(), this->box_.get(), &data),
        std::logic_error,
        "Requested a contact surface between a pair of geometries without "
        "hydroelastic representation .+ undefined .+ undefined .+");
  }

  // Case: Box geometry lacks representation.
  {
    Geometries hydroelastic_geometries;
    hydroelastic_geometries.MaybeAddGeometry(sphere, this->id_sphere_,
                                             rigid_properties());
    EXPECT_EQ(hydroelastic_geometries.hydroelastic_type(this->id_sphere_),
              HydroelasticType::kRigid);
    CallbackData<T> data(&this->collision_filter_, &this->X_WGs_,
                         &hydroelastic_geometries, &surfaces);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Callback<T>(this->sphere_.get(), this->box_.get(), &data),
        std::logic_error,
        "Requested a contact surface between a pair of geometries without "
        "hydroelastic representation .+ rigid .+ undefined .+");
  }

  // Case: Sphere geometry lacks representation.
  {
    Geometries hydroelastic_geometries;
    hydroelastic_geometries.MaybeAddGeometry(box, this->id_box_,
                                             rigid_properties());
    EXPECT_EQ(hydroelastic_geometries.hydroelastic_type(this->id_box_),
              HydroelasticType::kRigid);
    CallbackData<T> data(&this->collision_filter_, &this->X_WGs_,
                         &hydroelastic_geometries, &surfaces);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Callback<T>(this->sphere_.get(), this->box_.get(), &data),
        std::logic_error,
        "Requested a contact surface between a pair of geometries without "
        "hydroelastic representation .+ undefined .+ rigid .+");
  }
}

// Confirms that if the intersecting pair has the same compliance (rigid-rigid)
// or (soft-soft), that an exception is thrown. It is important that collisions
// are *not* filtered between the two objects.
TYPED_TEST(HydroelasticCallbackTyped, ThrowForBadPairBasedOnCompliance) {
  using T = TypeParam;

  const Sphere sphere(this->radius_);
  const Box box = Box::MakeCube(this->cube_size_);

  vector<ContactSurface<T>> surfaces;

  // Case: both geometries are rigid.
  {
    Geometries hydroelastic_geometries;
    hydroelastic_geometries.MaybeAddGeometry(sphere, this->id_sphere_,
                                             rigid_properties());
    hydroelastic_geometries.MaybeAddGeometry(box, this->id_box_,
                                             rigid_properties());
    CallbackData<T> data(&this->collision_filter_, &this->X_WGs_,
                         &hydroelastic_geometries, &surfaces);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Callback<T>(this->sphere_.get(), this->box_.get(), &data),
        std::logic_error,
        "Requested contact between two rigid objects .+ only rigid-soft pairs "
        "are currently supported");
  }

  // Case: both geometries are soft.
  {
    Geometries hydroelastic_geometries;
    hydroelastic_geometries.MaybeAddGeometry(sphere, this->id_sphere_,
                                             soft_properties());
    // TODO(SeanCurtis-TRI): When a soft box is supported, swap this for a box
    //  just to keep things interesting.
    hydroelastic_geometries.MaybeAddGeometry(sphere, this->id_box_,
                                             soft_properties());
    CallbackData<T> data(&this->collision_filter_, &this->X_WGs_,
                         &hydroelastic_geometries, &surfaces);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Callback<T>(this->sphere_.get(), this->box_.get(), &data),
        std::logic_error,
        "Requested contact between two soft objects .+ only rigid-soft pairs "
        "are currently supported");
  }
}

// Confirms that if the world contains unsupported geometry, as long as they are
// filtered, they don't pose a problem. The "lack of support" comes from the
// fact that the ids don't map to anything in the hydroelastic geometry set.
TYPED_TEST(HydroelasticCallbackTyped, RespectsCollisionFilter) {
  using T = TypeParam;

  // Assign both geometries to clique 1 -- this will cause the pair to be
  // filtered.
  EncodedData data_A(this->id_sphere_, true);
  EncodedData data_B(this->id_box_, true);
  this->collision_filter_.AddToCollisionClique(data_A.encoding(), 1);
  this->collision_filter_.AddToCollisionClique(data_B.encoding(), 1);

  // We leave the set of hydroelastic geometries empty on purpose. This implies
  // that *all* geometries are "unsupported" (i.e., have no hydroelastic
  // representation). It should not produce an error because the sphere and box
  // are filtered.
  Geometries hydroelastic_geometries;
  vector<ContactSurface<T>> surfaces;
  CallbackData<T> data(&this->collision_filter_, &this->X_WGs_,
                       &hydroelastic_geometries, &surfaces);
  DRAKE_EXPECT_NO_THROW(
      Callback<T>(this->sphere_.get(), this->box_.get(), &data));
  EXPECT_EQ(surfaces.size(), 0u);
}

// TODO(SeanCurtis-TRI): This test relies on the fact that all supported
//  geometry pairs pass through the same function (vol-surf mesh intersection).
//  When the calling path becomes more heterogeneous, this test will no longer
//  be sufficient.

// Confirms that a colliding collision pair (with supported hydroelastic
// representations) produces a result. This doesn't test the actual data -- it
// assumes the function responsible for computing that result has been
// successfully tested.
TYPED_TEST(HydroelasticCallbackTyped, ValidPairProducesResult) {
  using T = TypeParam;

  Geometries hydroelastic_geometries;
  hydroelastic_geometries.MaybeAddGeometry(
      Sphere(this->radius_), this->id_sphere_, soft_properties());
  hydroelastic_geometries.MaybeAddGeometry(
      Box(this->cube_size_, this->cube_size_, this->cube_size_), this->id_box_,
      rigid_properties());

  vector<ContactSurface<T>> surfaces;
  CallbackData<T> data(&this->collision_filter_, &this->X_WGs_,
                       &hydroelastic_geometries, &surfaces);
  DRAKE_EXPECT_NO_THROW(
      Callback<T>(this->sphere_.get(), this->box_.get(), &data));
  EXPECT_EQ(surfaces.size(), 1u);
}

}  // namespace
}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
