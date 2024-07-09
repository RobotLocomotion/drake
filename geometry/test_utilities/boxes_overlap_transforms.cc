#include "drake/geometry/test_utilities/boxes_overlap_transforms.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

RigidTransformd CalcCornerTransform(const Vector3d& a_half,
                                    const Vector3d& b_half, const int axis,
                                    const bool expect_overlap) {
  const int axis1 = (axis + 1) % 3;
  const int axis2 = (axis + 2) % 3;
  // Construct the rotation matrix, R_AB, that has meaningful (non-zero)
  // values everywhere for the remaining 2 axes and no symmetry.
  const RotationMatrixd R_AB =
      RotationMatrixd(AngleAxisd(M_PI / 5, Vector3d::Unit(axis1))) *
      RotationMatrixd(AngleAxisd(-M_PI / 5, Vector3d::Unit(axis2)));

  // We define p_BqBo in Frame A from box b's minimum corner Q to its center.
  const Vector3d p_BqBo_A = R_AB * b_half;
  // Reality check that the minimum corner and the center are strictly
  // increasing along the given axis because we chose the rotation R_AB to
  // support this property.
  DRAKE_DEMAND(p_BqBo_A[axis] > 0.);

  // We construct Bq to be a small relative offset either side of Af along the
  // given A[axis], depending on whether we expect the boxes to overlap.
  Vector3d p_AfBq_A{0, 0, 0};
  p_AfBq_A[axis] = expect_overlap ? -0.01 : 0.01;

  // We construct Af by taking the maximum corner and offsetting it along the
  // remaining 2 axes, e.g. by a quarter across. This ensures we thoroughly
  // exercise all bits instead of simply using any midpoints or corners.
  //
  //           A[axis1]
  //           ^
  //   --------|--------
  //   |       |       |
  //   |       |       Af
  //   |       |       |
  //   |      Ao------->A[axis]
  //   |               |
  //   |               |
  //   |               |
  //   -----------------
  //
  Vector3d p_AoAf_A = a_half;
  p_AoAf_A[axis1] /= 2;
  p_AoAf_A[axis2] /= 2;

  const Vector3d p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A;
  return RigidTransformd(R_AB, p_AoBo_A);
}

RigidTransformd CalcEdgeTransform(const Vector3d& a_half,
                                  const Vector3d& b_half, const int a_axis,
                                  const int b_axis, const bool expect_overlap) {
  const int a_axis1 = (a_axis + 1) % 3;
  const int a_axis2 = (a_axis + 2) % 3;
  const int b_axis1 = (b_axis + 1) % 3;
  const int b_axis2 = (b_axis + 2) % 3;
  // Construct a rotation matrix that has meaningful (non-zero) values
  // everywhere for the remaining 2 axes and no symmetry. Depending on the
  // combination of axes, we need to rotate around different axes to ensure
  // the edge remains as the minimum.
  RotationMatrixd R_AB;
  const double theta = M_PI / 5;
  // For cases Ax × Bx, Ay × By, and Az × Bz.
  if (a_axis == b_axis) {
    R_AB = RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis1))) *
           RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis2)));
    // For cases Ax × By, Ay × Bz, and Az × Bx.
  } else if (a_axis1 == b_axis) {
    R_AB = RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis1))) *
           RotationMatrixd(AngleAxisd(-theta, Vector3d::Unit(b_axis2)));
    // For cases Ax × Bz, Ay × Bx, and Az × By.
  } else {
    R_AB = RotationMatrixd(AngleAxisd(-theta, Vector3d::Unit(b_axis2))) *
           RotationMatrixd(AngleAxisd(theta, Vector3d::Unit(b_axis1)));
  }

  // We define p_BqBo in Frame B taking a point on the minimum edge aligned
  // with the given axis, offset it to be without symmetry, then convert it
  // to Frame A by applying the rotation.
  Vector3d p_BqBo_B = b_half;
  p_BqBo_B[b_axis] -= b_half[b_axis] / 2;
  const Vector3d p_BqBo_A = R_AB * p_BqBo_B;
  // Reality check that the point Bq and the center Bo are strictly
  // increasing along the remaining 2 axes because we chose the rotation R_AB
  // to support this property.
  DRAKE_DEMAND(p_BqBo_A[a_axis1] > 0);
  DRAKE_DEMAND(p_BqBo_A[a_axis2] > 0);

  // We construct Bq to be a small relative offset either side of Af along the
  // given axis, depending on whether we expect the boxes to overlap.
  Vector3d p_AfBq_A{0, 0, 0};
  const double offset = expect_overlap ? -0.01 : 0.01;
  p_AfBq_A[a_axis1] = offset;
  p_AfBq_A[a_axis2] = offset;

  // We construct Af by taking the maximum corner and offsetting it along the
  // given edge to thoroughly exercise all bits.
  Vector3d p_AoAf_A = a_half;
  p_AoAf_A[a_axis] -= a_half[a_axis] / 2;

  Vector3d p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A;
  // Finally we combine the components to form the transform X_AB.
  return RigidTransformd(R_AB, p_AoBo_A);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
