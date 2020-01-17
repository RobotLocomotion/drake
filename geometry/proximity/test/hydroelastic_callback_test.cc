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
//  into the type list: ScalarTypes.

// Infrastructure to repeat tests on both double (and, someday, AutoDiffXd).
using ScalarTypes = ::testing::Types<double>;

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

  vector<PenetrationAsPointPair<double>> point_pairs;
  CallbackWithFallbackData<AutoDiffXd> fallback_data{
      {&collision_filter, &X_WGs, &hydroelastic_geometries, &surfaces},
      &point_pairs};
  DRAKE_EXPECT_THROWS_MESSAGE(
      CallbackWithFallback<AutoDiffXd>(&object_A, &object_B, &fallback_data),
      std::logic_error,
      "AutoDiff-valued ContactSurface calculation between meshes is not"
      "currently supported");
}

// Utility class for encoding a scene with two spheres.
template <typename T>
class TestScene {
 public:
  TestScene()
      : data_{&collision_filter_, &X_WGs_, &hydroelastic_geometries_,
              &surfaces_} {}

  // Configures a scene with two spheres. Each is given a hydroelastic
  // representation based on the given requested types and are positioned in
  // a colliding state (if `are_colliding` is true).
  void ConfigureScene(const HydroelasticType type_A,
                      const HydroelasticType type_B,
                      bool are_colliding = true) {
    // Configures a world with a sphere slightly overlapping a box.
    id_A_ = GeometryId::get_new_id();
    id_B_ = GeometryId::get_new_id();
    EncodedData data_A(id_A_, true);
    EncodedData data_B(id_B_, true);
    collision_filter_.AddGeometry(data_A.encoding());
    collision_filter_.AddGeometry(data_B.encoding());
    // Leave sphere B centered on origin.
    X_WGs_.insert({id_B_, RigidTransform<T>::Identity()});

    // The spheres will either be colliding to a depth of radius_ / 5 or
    // separated by the same distance based on `are_colliding`. Sphere B stays
    // at the origin, and sphere A is moved along the +z axis the appropriate
    // displacement.
    const double z_offset = radius_ * (are_colliding ? 1.8 : 2.2);
    X_WGs_.insert({id_A_, RigidTransform<T>(Vector3<T>{0, 0, z_offset})});

    sphere_A_ = make_unique<CollisionObjectd>(make_shared<Sphered>(radius_));
    data_A.write_to(sphere_A_.get());
    sphere_B_ = make_unique<CollisionObjectd>(make_shared<Sphered>(radius_));
    data_B.write_to(sphere_B_.get());

    auto add_hydroelastic = [this](GeometryId id, const HydroelasticType type) {
      if (type == HydroelasticType::kSoft) {
        this->hydroelastic_geometries_.MaybeAddGeometry(Sphere(this->radius_),
                                                        id, soft_properties());
      } else if (type == HydroelasticType::kRigid) {
        this->hydroelastic_geometries_.MaybeAddGeometry(Sphere(this->radius_),
                                                        id, rigid_properties());
      }
      // Note: HydroelasticType::kUndefined will not be added.
    };

    add_hydroelastic(id_A_, type_A);
    add_hydroelastic(id_B_, type_B);
  }

  // Filters contact between the two spheres.
  void FilterContact() {
    EncodedData data_A(*sphere_A_);
    EncodedData data_B(*sphere_B_);
    collision_filter_.AddToCollisionClique(data_A.encoding(), 1);
    collision_filter_.AddToCollisionClique(data_B.encoding(), 1);
  }

  // Note: these are non const because the callback takes non-const pointers
  // (due to FCL's API).
  CollisionObjectd& sphere_A() { return *sphere_A_; }
  CollisionObjectd& sphere_B() { return *sphere_B_; }
  CallbackData<T>& data() { return data_; }
  const vector<ContactSurface<T>>& surfaces() const { return surfaces_; }

 private:
  Geometries hydroelastic_geometries_;
  CollisionFilterLegacy collision_filter_;
  unordered_map<GeometryId, RigidTransform<T>> X_WGs_;
  GeometryId id_A_{};
  GeometryId id_B_{};
  const double radius_{0.25};
  unique_ptr<CollisionObjectd> sphere_A_;
  unique_ptr<CollisionObjectd> sphere_B_;
  vector<ContactSurface<T>> surfaces_;
  CallbackData<T> data_;
};

TYPED_TEST_SUITE(MaybeCalcContactSurfaceTests, ScalarTypes);

// Test suite for exercising MaybeCalcContactSurface with different scalar
// types. (Currently only double as the hydroelastic infrastructure doesn't
// support autodiff yet.)
template <typename T>
class MaybeCalcContactSurfaceTests : public ::testing::Test {};

// Confirms that if one or both geometries do not have a hydroelastic
// representation, that the proper result is returned.
TYPED_TEST(MaybeCalcContactSurfaceTests, UndefinedGeometry) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kUndefined);

  // Case: second is undefined.
  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.sphere_A(), &scene.sphere_B(), &scene.data());
  ASSERT_EQ(result, CalcContactSurfaceResult::kUnsupported);
  ASSERT_EQ(scene.surfaces().size(), 0u);

  // Case: first is undefined.
  result = MaybeCalcContactSurface<T>(
      &scene.sphere_B(), &scene.sphere_A(), &scene.data());
  ASSERT_EQ(result, CalcContactSurfaceResult::kUnsupported);
  ASSERT_EQ(scene.surfaces().size(), 0u);

  // Case: both are undefined.
  result = MaybeCalcContactSurface<T>(
      &scene.sphere_B(), &scene.sphere_B(), &scene.data());
  ASSERT_EQ(result, CalcContactSurfaceResult::kUnsupported);
  ASSERT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that matching compliance (rigid) can't be evaluated.
TYPED_TEST(MaybeCalcContactSurfaceTests, BothRigid) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);

  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.sphere_A(), &scene.sphere_B(), &scene.data());
  EXPECT_EQ(result, CalcContactSurfaceResult::kSameCompliance);
  EXPECT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that matching compliance (soft) can't be evaluated.
TYPED_TEST(MaybeCalcContactSurfaceTests, BothSoft) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kSoft, HydroelasticType::kSoft);

  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.sphere_A(), &scene.sphere_B(), &scene.data());
  EXPECT_EQ(result, CalcContactSurfaceResult::kSameCompliance);
  EXPECT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that a valid pair that are, nevertheless, not colliding does not
// add a surface to the results.
TYPED_TEST(MaybeCalcContactSurfaceTests, NonColliding) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kSoft, HydroelasticType::kRigid,
                       false /* are_colliding */);

  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.sphere_A(), &scene.sphere_B(), &scene.data());
  EXPECT_EQ(result, CalcContactSurfaceResult::kCalculated);
  EXPECT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that a valid pair of hydroelastic representations report as such
// and produce a result. (The details of the result are not explicitly
// evaluated as they have been tested by the underlying method's unit tests.)
TYPED_TEST(MaybeCalcContactSurfaceTests, ValidCompliancePair) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kSoft);

  CalcContactSurfaceResult result = MaybeCalcContactSurface<T>(
      &scene.sphere_A(), &scene.sphere_B(), &scene.data());
  // TODO(SeanCurtis-TRI): This test relies on the fact that all supported
  //  geometry pairs use the same function (vol-surf mesh intersection). When
  //  the calling path becomes more heterogeneous, this test will no longer
  //  be sufficient.
  EXPECT_EQ(result, CalcContactSurfaceResult::kCalculated);
  EXPECT_EQ(scene.surfaces().size(), 1u);
}

TYPED_TEST_SUITE(StrictHydroelasticCallbackTyped, ScalarTypes);

// Test infrastructure for the strict hydroelastic callback for arbitrary
// scalar type. It makes use of MaybeCalculationContactSurface() but this method
// has the following responsibilities:
//   - invoke the method using the data provided to it (so that the results
//     ultimately percolate outward).
//   - throw on any return value that isn't wholly successful.
//   - respect collision filtering.
// (Currently only double as the hydroelastic infrastructure doesn't support
// autodiff yet.)
template <typename T>
class StrictHydroelasticCallbackTyped : public ::testing::Test {};

// Confirms that if the intersecting pair is missing hydroelastic representation
// that an exception is thrown. This test applies *no* collision filters
// to guarantee that the body of the callback gets exercised in all cases.
TYPED_TEST(StrictHydroelasticCallbackTyped,
           ThrowForMissingHydroelasticRepresentation) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kUndefined);

  // We test only a single "underrepresented" configuration (rigid, undefined)
  // because we rely on the tests on MaybeCalcContactSurface() to have explored
  // all the ways that the kUnsupported calculation result is returned. This
  // configuration is representative of that set.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Callback<T>(&scene.sphere_A(), &scene.sphere_B(), &scene.data()),
      std::logic_error,
      "Requested a contact surface between a pair of geometries without "
      "hydroelastic representation .+ rigid .+ undefined .+");
}

// Confirms that if the intersecting pair has the same compliance (rigid-rigid)
// or (soft-soft), that an exception is thrown. This test applies *no* collision
// filters to guarantee that the body of the callback gets exercised in all
// cases.
TYPED_TEST(StrictHydroelasticCallbackTyped, ThrowForSameComplianceType) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);

  // We test only a single "same-compliance" configuration (rigid, rigid)
  // because we rely on the tests on MaybeCalcContactSurface() to have explored
  // all the ways that the kSameCompliance calculation result is returned. This
  // configuration is representative of that set.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Callback<T>(&scene.sphere_A(), &scene.sphere_B(), &scene.data()),
      std::logic_error,
      "Requested contact between two rigid objects .+");
}

// Confirms that if the pair contains unsupported geometry, as long as they are
// filtered, they don't pose a problem. The "lack of support" comes from the
// fact that the ids don't map to anything in the hydroelastic geometry set.
TYPED_TEST(StrictHydroelasticCallbackTyped, RespectsCollisionFilter) {
  using T = TypeParam;

  TestScene<T> scene;
  // Note: a configuration that would cause an exception to be thrown if
  // unfiltered and the confirmation of that assumption.
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);
  EXPECT_THROW(Callback<T>(&scene.sphere_A(), &scene.sphere_B(), &scene.data()),
               std::logic_error);

  scene.FilterContact();
  DRAKE_EXPECT_NO_THROW(
      Callback<T>(&scene.sphere_A(), &scene.sphere_B(), &scene.data()));
  EXPECT_EQ(scene.surfaces().size(), 0u);
}

// Confirms that a colliding collision pair (with supported hydroelastic
// representations) produces a result. This doesn't test the actual data -- it
// assumes the function responsible for computing that result has been
// successfully tested. This test is subtle; it simply confirms that the
// Callback invokes MaybeCalcContactSurface() and provides the correct
// vector<ContactSurface> instance.
TYPED_TEST(StrictHydroelasticCallbackTyped, ValidPairProducesResult) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kSoft);

  DRAKE_EXPECT_NO_THROW(
      Callback<T>(&scene.sphere_A(), &scene.sphere_B(), &scene.data()));
  EXPECT_EQ(scene.surfaces().size(), 1u);
}

TYPED_TEST_SUITE(HydroelasticCallbackFallbackTyped, ScalarTypes);

// Test infrastructure for the hydroelastic callback with fallback for arbitrary
// scalar type. It makes use of MaybeCalculationContactSurface() but this method
// has the following responsibilities:
//   - invoke the method using the data provided to it (so that the results
//     ultimately percolate outward).
//   - Compute point pair penetration for any pair that couldn't be calculated.
//   - respect collision filtering.
// (Currently only double as the hydroelastic infrastructure doesn't support
// autodiff yet.)
// The tests below parallel the tests for the strict hydroelastic callback.
template <typename T>
class HydroelasticCallbackFallbackTyped : public ::testing::Test {};

// Confirms that if the intersecting pair is missing hydroelastic representation
// a point pair is generated. This test applies *no* collision filters
// to guarantee that the body of the callback gets exercised in all cases.
TYPED_TEST(HydroelasticCallbackFallbackTyped,
           PointPairForMissingHydroelasticRepresentation) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kUndefined);

  // We test only a single "underrepresented" configuration (rigid, undefined)
  // because we rely on the tests on MaybeCalcContactSurface() to have explored
  // all the ways that the kUnsupported calculation result is returned. This
  // configuration is representative of that set.
  vector<PenetrationAsPointPair<T>> point_pairs;
  CallbackWithFallbackData<T> data{scene.data(), &point_pairs};
  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.sphere_A(), &scene.sphere_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 0u);
  EXPECT_EQ(point_pairs.size(), 1u);
}

// Confirms that if the intersecting pair has the same compliance (rigid-rigid)
// or (soft-soft), that we return a point-pair. This test applies *no* collision
// filters to guarantee that the body of the callback gets exercised in all
// cases.
TYPED_TEST(HydroelasticCallbackFallbackTyped, PointPairForSameComplianceType) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);

  // We test only a single "same-compliance" configuration (rigid, rigid)
  // because we rely on the tests on MaybeCalcContactSurface() to have explored
  // all the ways that the kSameCompliance calculation result is returned. This
  // configuration is representative of that set.
  vector<PenetrationAsPointPair<T>> point_pairs;
  CallbackWithFallbackData<T> data{scene.data(), &point_pairs};
  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.sphere_A(), &scene.sphere_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 0u);
  EXPECT_EQ(point_pairs.size(), 1u);
}

// Confirms that collision filters are respected; for a filtered colliding pair,
// no results are returned at all.
TYPED_TEST(HydroelasticCallbackFallbackTyped, RespectsCollisionFilter) {
  using T = TypeParam;

  TestScene<T> scene;
  // Note: a configuration that would cause an exception to be thrown if
  // unfiltered and the confirmation of that assumption.
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kRigid);

  vector<PenetrationAsPointPair<T>> point_pairs;
  CallbackWithFallbackData<T> data{scene.data(), &point_pairs};
  // Confirm collision state.
  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.sphere_A(), &scene.sphere_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 0u);
  EXPECT_EQ(point_pairs.size(), 1u);

  // Now filter.
  point_pairs.clear();
  scene.FilterContact();

  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.sphere_A(), &scene.sphere_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 0u);
  EXPECT_EQ(point_pairs.size(), 0u);
}

// Confirms that a colliding collision pair (with supported hydroelastic
// representations) produces a contact surface. This doesn't test the actual
// data -- it assumes the function responsible for computing that result has
// been successfully tested. This test is subtle; it simply confirms that the
// Callback invokes MaybeCalcContactSurface() and provides the correct
// vector<ContactSurface> instance.
TYPED_TEST(HydroelasticCallbackFallbackTyped, ValidPairProducesResult) {
  using T = TypeParam;

  TestScene<T> scene;
  scene.ConfigureScene(HydroelasticType::kRigid, HydroelasticType::kSoft);

  vector<PenetrationAsPointPair<T>> point_pairs;
  CallbackWithFallbackData<T> data{scene.data(), &point_pairs};

  DRAKE_EXPECT_NO_THROW(
      CallbackWithFallback<T>(&scene.sphere_A(), &scene.sphere_B(), &data));
  EXPECT_EQ(scene.surfaces().size(), 1u);
  EXPECT_EQ(point_pairs.size(), 0u);
}

}  // namespace
}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
