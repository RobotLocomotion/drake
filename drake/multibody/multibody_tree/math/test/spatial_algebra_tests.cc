#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Matrix3d;

GTEST_TEST(SpatialAlgebra, SpatialVelocityShift) {
  // Linear velocity of frame B measured and expressed in frame A.
  Vector3d v_AB(1, 2, 0);

  // Angular velocity of frame B measured and expressed in frame A.
  Vector3d w_AB(0, 0, 3);

  // Spatial velocity of frame B with respect to A and expressed in A.
  SpatialVector<double> V_AB(w_AB, v_AB);

  // A shift operator representing the rigid body transformation from frame B
  // to frame Q, expressed in A.
  ShiftOperator<double> phi_BQ_A({2, -2, 0});

  SpatialVector<double> V_AQ = phi_BQ_A.transpose() * V_AB;
  SpatialVector<double> expected_V_AQ(w_AB, {7, 8, 0});

  EXPECT_TRUE(V_AQ.angular().isApprox(expected_V_AQ.angular()));
  EXPECT_TRUE(V_AQ.linear().isApprox(expected_V_AQ.linear()));
}

}
}  // math
}  // multibody
}  // drake