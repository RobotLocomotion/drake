#include "drake/geometry/proximity/hydroelastic_callback.h"

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
using std::make_shared;
using std::vector;

// Infrastructure to repeat tests on both double and AutoDiffXd.
using ScalarTypes = ::testing::Types<double, AutoDiffXd>;

template <typename T>
class HydroelasticCallbackTyped : public ::testing::Test {};

TYPED_TEST_CASE(HydroelasticCallbackTyped, ScalarTypes);

// Confirms that sphere-halfspace is supported for double and autodiff, but only
// for sphere and halfspace. If support fractures across scalar type, this will
// need to be split.
TYPED_TEST(HydroelasticCallbackTyped, ScalarSupport) {
  using Support = ScalarSupport<TypeParam>;
  EXPECT_TRUE(Support::is_supported(fcl::GEOM_SPHERE, fcl::GEOM_HALFSPACE));
  EXPECT_TRUE(Support::is_supported(fcl::GEOM_HALFSPACE, fcl::GEOM_SPHERE));

  // TODO(SeanCurtis-TRI): Add other supported fcl geometry types as they get
  // introduced (here and below for autodiff support).
  for (auto other : {fcl::GEOM_BOX, fcl::GEOM_CONVEX, fcl::GEOM_CYLINDER}) {
    EXPECT_FALSE(Support::is_supported(fcl::GEOM_SPHERE, other));
    EXPECT_FALSE(Support::is_supported(other, fcl::GEOM_SPHERE));
    EXPECT_FALSE(Support::is_supported(other, fcl::GEOM_HALFSPACE));
    EXPECT_FALSE(Support::is_supported(fcl::GEOM_HALFSPACE, other));
  }
}

// TODO(SeanCurtis-TRI): Refactor the "set up CallbackData" boilerplate across
//  these three (or more) tests.

// Confirms that if geometry to be collided are not supported, that an exception
// is thrown.
TYPED_TEST(HydroelasticCallbackTyped, ThrowForUnsupported) {
  using T = TypeParam;
  std::vector<GeometryId> geometry_map{GeometryId::get_new_id(),
                                       GeometryId::get_new_id()};
  CollisionFilterLegacy collision_filter;

  EncodedData data_A(GeometryIndex{0}, true);
  EncodedData data_B(GeometryIndex{1}, true);
  collision_filter.AddGeometry(data_A.encoding());
  collision_filter.AddGeometry(data_B.encoding());
  std::vector<Isometry3<T>> X_WGs{Isometry3<T>::Identity(),
                                  Isometry3<T>::Identity()};

  CollisionObjectd box_A(make_shared<Boxd>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  CollisionObjectd box_B(make_shared<Boxd>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  vector<ContactSurface<T>> surfaces;
  CallbackData<T> data(&geometry_map, &collision_filter, &X_WGs, &surfaces);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Callback<T>(&box_A, &box_B, &data), std::logic_error,
      "Contact surface queries between .* and .* are not supported .*");
}

// Confirms that if the world contains unsupported geometry, as long as they are
// filtered, they don't pose a problem.
TYPED_TEST(HydroelasticCallbackTyped, RespectsCollisionFilter) {
  using T = TypeParam;
  std::vector<GeometryId> geometry_map{GeometryId::get_new_id(),
                                       GeometryId::get_new_id()};
  CollisionFilterLegacy collision_filter;

  EncodedData data_A(GeometryIndex{0}, true);
  EncodedData data_B(GeometryIndex{1}, true);
  collision_filter.AddGeometry(data_A.encoding());
  collision_filter.AddGeometry(data_B.encoding());
  // Filter the pair (A, B) by adding them to the same clique.
  collision_filter.AddToCollisionClique(data_A.encoding(), 1);
  collision_filter.AddToCollisionClique(data_B.encoding(), 1);
  std::vector<Isometry3<T>> X_WGs{Isometry3<T>::Identity(),
                                  Isometry3<T>::Identity()};

  CollisionObjectd box_A(make_shared<Boxd>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  CollisionObjectd box_B(make_shared<Boxd>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  vector<ContactSurface<T>> surfaces;
  CallbackData<T> data(&geometry_map, &collision_filter, &X_WGs, &surfaces);
  EXPECT_NO_THROW(Callback<T>(&box_A, &box_B, &data));
  EXPECT_EQ(surfaces.size(), 0u);
}

// Confirms that a colliding sphere and halfspace produces a result. This
// doesn't test the actual data -- it assumes the function responsible for
// computing that result has been successfully tested.
TYPED_TEST(HydroelasticCallbackTyped, SphereHalfspaceProducesResult) {
  using T = TypeParam;
  std::vector<GeometryId> geometry_map{GeometryId::get_new_id(),
                                       GeometryId::get_new_id()};
  CollisionFilterLegacy collision_filter;

  EncodedData data_A(GeometryIndex{0}, true);
  EncodedData data_B(GeometryIndex{1}, true);
  collision_filter.AddGeometry(data_A.encoding());
  collision_filter.AddGeometry(data_B.encoding());
  std::vector<Isometry3<T>> X_WGs{Isometry3<T>::Identity(),
                                  Isometry3<T>::Identity()};

  CollisionObjectd sphere(make_shared<Sphered>(0.5));
  data_A.write_to(&sphere);
  CollisionObjectd halfspace(make_shared<Halfspaced>(Vector3d{0, 0, 1}, 0));
  data_B.write_to(&halfspace);

  vector<ContactSurface<T>> surfaces;
  CallbackData<T> data(&geometry_map, &collision_filter, &X_WGs, &surfaces);
  EXPECT_NO_THROW(Callback<T>(&sphere, &halfspace, &data));
  EXPECT_EQ(surfaces.size(), 1u);
}

}  // namespace
}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
