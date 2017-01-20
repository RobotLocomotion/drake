#include "drake/multibody/collision/fcl_model.h"

#include <fcl/fcl.h>
#include <gtest/gtest.h>

namespace DrakeCollision {
namespace {

// TODO(jamiesnape): Test the model.

// TODO(jamiesnape): Remove this test as redundant once model is implemented.
GTEST_TEST(FCLModelTest, SphereSphereDistance) {
  const fcl::Sphere<double> sphere0(20.0);
  fcl::Transform3<double> transform0;
  transform0.setIdentity();

  const fcl::Sphere<double> sphere1(10.0);
  fcl::Transform3<double> transform1;
  transform1.setIdentity();
  transform1.translation() << 40.0, 0.0, 0.0;

  fcl::DistanceRequest<double> distance_request;
  distance_request.enable_signed_distance = true;
  distance_request.gjk_solver_type = fcl::GST_LIBCCD;

  fcl::DistanceResult<double> distance_result;

  EXPECT_TRUE(fcl::distance(&sphere0, transform0, &sphere1, transform1,
                            distance_request, distance_result));
  EXPECT_DOUBLE_EQ(10.0, distance_result.min_distance);

  transform1.translation() << 25.0, 0.0, 0.0;
  distance_result.clear();

  EXPECT_TRUE(fcl::distance(&sphere0, transform0, &sphere1, transform1,
                            distance_request, distance_result));
  EXPECT_DOUBLE_EQ(-5.0, distance_result.min_distance);
}

}  // namespace
}  // namespace DrakeCollision
