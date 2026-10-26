#include "drake/geometry/proximity/test/backend_utilities.h"

#include <gtest/gtest.h>

#include "drake/geometry/proximity/coal_backend.h"
#include "drake/geometry/proximity/fcl_backend.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

template <typename Backend>
class BackendUtilitiesTest : public ::testing::Test {};

using Backends = ::testing::Types<FclBackend, CoalBackend>;
TYPED_TEST_SUITE(BackendUtilitiesTest, Backends);

TYPED_TEST(BackendUtilitiesTest, MakeBackendObject) {
  using Backend = TypeParam;
  const GeometryId id = GeometryId::get_new_id();
  const math::RigidTransformd X_WG(math::RollPitchYawd(0.1, 0.2, 0.3),
                                   Eigen::Vector3d(1.0, 2.0, 3.0));

  const auto object = MakeBackendObject<Backend>(Box(2.0, 4.0, 6.0), id,
                                                 /* is_dynamic= */ true, X_WG);

  const EncodedData data(*object);
  EXPECT_EQ(data.id(), id);
  EXPECT_TRUE(data.is_dynamic());
  EXPECT_TRUE(Backend::FromTransform(object->getTransform())
                  .IsNearlyEqualTo(X_WG, 1e-14));

  const typename Backend::Geometry* geometry =
      object->collisionGeometry().get();
  ASSERT_EQ(Backend::Classify(geometry->getNodeType()), ShapeKind::kBox);
  const auto& box = static_cast<const typename Backend::Box&>(*geometry);
  EXPECT_EQ(Backend::half_extents(box), Eigen::Vector3d(1.0, 2.0, 3.0));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
