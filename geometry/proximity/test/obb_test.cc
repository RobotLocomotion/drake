#include "drake/geometry/proximity/obb.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

// Friend class for accessing AAbb's protected/private functionality.
class ObbTester : public ::testing::Test {
 public:
  static constexpr double kTolerance = Obb::kTolerance;
};

// Friend class for accessing ObbMaker's private functionality for testing.
template <typename MeshType>
class ObbMakerTester {
 public:
  ObbMakerTester(const MeshType& mesh_M,
                 const std::set<typename MeshType::VertexIndex>& vertices)
      : obb_maker_(mesh_M, vertices) {}

  RotationMatrixd CalcOrientationByPCA() const {
    return obb_maker_.CalcOrientationByPCA();
  }

  Obb CalcOrientedBox(const math::RotationMatrixd& R_MB) const {
    return obb_maker_.CalcOrientedBox(R_MB);
  }

  // CalcVolumeGradient() is also tested as a part of OptimizeObbVolume().
  Obb OptimizeObbVolume(const Obb& box) const {
    return obb_maker_.OptimizeObbVolume(box);
  }

 private:
  ObbMaker<MeshType> obb_maker_;
};

namespace {

// Test fixture class for testing ObbMaker::CalcOrientationByPCA() and
// ObbMaker::CalcOrientedBox(). We use a simple example that we know
// a non-trivial expected result.
//
// The test example consists of vertices of an equilateral triangle.
// Each vertex is at unit distance from the origin along each axis
// of frame M like this:
//
//            Mz
//            |
//            v2
//            |
//            |
//            +-----v1--My
//           /
//          /
//         v0
//        /
//      Mx
//
class ObbMakerTestTriangle : public ::testing::Test {
 public:
  ObbMakerTestTriangle()
      : ::testing::Test(),
        mesh_M_({SurfaceFace(SurfaceVertexIndex(0), SurfaceVertexIndex(1),
                             SurfaceVertexIndex(2))},
                {SurfaceVertex<double>(Vector3d::UnitX()),
                 SurfaceVertex<double>(Vector3d::UnitY()),
                 SurfaceVertex<double>(Vector3d::UnitZ())}),
        test_vertices_{SurfaceVertexIndex(0), SurfaceVertexIndex(1),
                       SurfaceVertexIndex(2)} {}

 protected:
  const SurfaceMesh<double> mesh_M_;
  const std::set<SurfaceVertexIndex> test_vertices_;
};

TEST_F(ObbMakerTestTriangle, CalcOrientationByPCA) {
  const RotationMatrixd R_MB =
      ObbMakerTester<SurfaceMesh<double>>(mesh_M_, test_vertices_)
          .CalcOrientationByPCA();

  const RotationMatrixd expect_R_MB =
      RotationMatrixd::MakeFromOrthonormalColumns(
          Vector3d(1., -0.5, -0.5).normalized(),
          Vector3d(0., 1., -1.).normalized(),
          Vector3d(1., 1., 1.).normalized());

  // Test tolerance is estimated from construction of covariance matrix and
  // eigen solver.
  const double kEps = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(R_MB.matrix(), expect_R_MB.matrix(),
                              kEps));
}

TEST_F(ObbMakerTestTriangle, CalcOrientedBox) {
  const RotationMatrixd R_MB =
      RotationMatrixd::MakeFromOrthonormalColumns(
          Vector3d(1., -0.5, -0.5).normalized(),
          Vector3d(0., 1., -1.).normalized(),
          Vector3d(1., 1., 1.).normalized());

  const Obb obb = ObbMakerTester<SurfaceMesh<double>>(mesh_M_, test_vertices_)
                      .CalcOrientedBox(R_MB);

  // Check the input R_MB passes to the output obb exactly.
  EXPECT_TRUE(
      CompareMatrices(obb.pose().rotation().matrix(), R_MB.matrix()));

  // Check the output center of the box. Note that the center(1/2,1/4,1/4) of
  // the box is *not* the centroid(1/3,1/3,1/3) of the triangle.
  const Vector3d expect_p_MB(0.5, 0.25, 0.25);
  EXPECT_TRUE(CompareMatrices(obb.center(), expect_p_MB,
                              std::numeric_limits<double>::epsilon()));

  // Check the output half-width vector. The extra tolerance term was
  // introduced by padding (see Obb::PadBoundary()).
  const Vector3d expect_half_width =
      Vector3d(std::sqrt(6.) / 4, 1. / std::sqrt(2.), 0.) +
      Vector3d::Constant(ObbTester::kTolerance);
  EXPECT_TRUE(CompareMatrices(obb.half_width(), expect_half_width,
                              std::numeric_limits<double>::epsilon()));
}

// Returns true iff the oriented bounding box contains all the specified
// vertices in the mesh.
template <typename MeshType>
bool Contain(const Obb& obb_M, const MeshType& mesh_M,
             const std::set<typename MeshType::VertexIndex>& vertices) {
  const RigidTransformd& X_MB = obb_M.pose();
  const RigidTransformd X_BM = X_MB.inverse();
  for (const typename MeshType::VertexIndex v : vertices) {
    Vector3d p_MV = mesh_M.vertex(v).r_MV();
    Vector3d p_BV = X_BM * p_MV;
    if ((p_BV.array() > obb_M.half_width().array()).any()) {
      return false;
    }
    if ((p_BV.array() < -obb_M.half_width().array()).any()) {
      return false;
    }
  }
  return true;
}

GTEST_TEST(ObbMakerTest, TestOptimizeObbVolume) {
  // Create an octahedron that is anisotropic (different lengths in different
  // axes) via a coarse mesh of an ellipsoid. Then, transform vertices to
  // a general pose.
  SurfaceMesh<double> mesh_M =
      MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1., 2., 3.), 6);
  const RigidTransformd transform_in_place(
      RotationMatrixd(RollPitchYawd(2. * M_PI / 3., M_PI / 4., M_PI / 5.)),
      Vector3d(-0.2, 2, 1.5));
  mesh_M.TransformVertices(transform_in_place);
  // Confirm that it is an octahedron.
  ASSERT_EQ(8, mesh_M.num_faces());
  ASSERT_EQ(6, mesh_M.num_vertices());
  const std::set<SurfaceVertexIndex> test_vertices{
      SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2),
      SurfaceVertexIndex(3), SurfaceVertexIndex(4), SurfaceVertexIndex(5)};

  // Initial obb is aligned with the three axes of the ellipsoid.
  const Obb initial_obb(transform_in_place, Vector3d(1., 2., 3.));
  const double initial_volume = initial_obb.CalcVolume();
  ASSERT_TRUE(Contain(initial_obb, mesh_M, test_vertices));

  const Obb optimized_obb =
      ObbMakerTester<SurfaceMesh<double>>(mesh_M, test_vertices)
          .OptimizeObbVolume(initial_obb);
  EXPECT_TRUE(Contain(optimized_obb, mesh_M, test_vertices));
  const double percent_improvement =
      100. * (initial_volume - optimized_obb.CalcVolume()) / initial_volume;
  // Empirically we saw about 22% improvement for this particular example.
  // If we change the optimization algorithm or modify the example, the number
  // might change.
  EXPECT_GT(percent_improvement, 22.0);
  EXPECT_LT(percent_improvement, 23.0);
}

// TODO(DamrongGuoy): Update the tests when we improve the algorithm
//  for computing oriented bounding boxes.  Currently it is not clear how
//  optimal the algorithm is.

// Tests computing an oriented bounding box of a vertex set of an octahedron.
// It is not easy to predict the pose and size of the expected bounding box,
// so we check the result indirectly in two steps:
// 1. Check that the box contains all the specified vertices.
// 2. Check that the box has volume around an empirically determined threshold.
//
// We also show that our algorithm is not always optimal. We use the same
// octahedron at two different poses and get two boxes with very different
// sizes.
GTEST_TEST(ComputeObbTest, TestOctahedron) {
  // Use a coarse sphere, i.e. an octahedron, as the underlying mesh.
  auto surface_mesh = MakeSphereSurfaceMesh<double>(Sphere(1.5), 3);
  // Verify that it is an octahedron.
  ASSERT_EQ(8, surface_mesh.num_faces());
  ASSERT_EQ(6, surface_mesh.num_vertices());

  // Add all the vertices from the octahedron for testing.
  const std::set<SurfaceVertexIndex> test_vertices{
      SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2),
      SurfaceVertexIndex(3), SurfaceVertexIndex(4), SurfaceVertexIndex(5)};
  Obb obb = ComputeObb<SurfaceMesh<double>>(surface_mesh, test_vertices);
  EXPECT_TRUE(Contain(obb, surface_mesh, test_vertices));
  // The threshold was set empirically . It is slightly smaller than the
  // trivial upper bound 27.0, which is the volume of a cube bounding the
  // sphere. It is possible to do better as shown in the next test below.
  EXPECT_LT(obb.CalcVolume(), 26.997);
  EXPECT_GT(obb.CalcVolume(), 26.996);

  // TODO(DamrongGuoy): Investigate why we get a tighter box when we change
  //  the pose of the mesh below. Perhaps the optimization didn't work well
  //  in the previous test due to the symmetric configuration? If we can
  //  improve the algorithm to work well for both cases, we can keep only
  //  one test.

  // Transform vertices to a general pose to avoid symmetry and show that the
  // current algorithm can give a box with very different size.
  surface_mesh.TransformVertices(RigidTransformd(
      RotationMatrixd(RollPitchYawd(M_PI / 6., M_PI / 3., M_PI / 5.)),
      Vector3d(-0.2, 2, 1.5)));
  obb = ComputeObb<SurfaceMesh<double>>(surface_mesh, test_vertices);
  EXPECT_TRUE(Contain(obb, surface_mesh, test_vertices));
  // The threshold was set empirically. It is much smaller than the
  // previous test before vertex transform.
  EXPECT_LT(obb.CalcVolume(), 9.741);
  EXPECT_GT(obb.CalcVolume(), 9.740);
}

// Smoke test that it works with VolumeMesh<double>.
GTEST_TEST(ComputeObbTest, TestVolumeMesh) {
  // Use a very coarse mesh.
  const VolumeMesh<double> volume_mesh = MakeEllipsoidVolumeMesh<double>(
      Ellipsoid(1., 2., 3.), 6, TessellationStrategy::kSingleInteriorVertex);
  std::set<VolumeVertexIndex> test_vertices;
  for (VolumeVertexIndex i(0); i < volume_mesh.num_vertices(); ++i) {
    test_vertices.insert(i);
  }
  Obb obb = ComputeObb<VolumeMesh<double>>(volume_mesh, test_vertices);
  EXPECT_TRUE(Contain(obb, volume_mesh, test_vertices));
  // The threshold was set empirically. It is slightly smaller than the
  // trivial upper bound 48.0, which is the volume of the box of size 2x4x6
  // that fits the ellipsoid.
  EXPECT_LT(obb.CalcVolume(), 41.317);
  EXPECT_GT(obb.CalcVolume(), 41.316);
}

// Tests calculating the bounding box volume. Due to boundary padding, the
// volume is increased from 8abc to 8((a + ε)*(b + ε)*(c+ε)), i.e.:
// 8[abc + (ab + bc + ac)ε + (a + b + c)ε² + ε³].
TEST_F(ObbTester, TestVolume) {
  Obb obb = Obb(RigidTransformd(Vector3d(-1, 2, 1)), Vector3d(2, 0.5, 2.7));
  // In this case the dominating error term is 8(ab + bc + ac)ε, which caps
  // out under kTolerance * 70.
  const double volume = obb.CalcVolume();
  EXPECT_NEAR(volume, 21.6, kTolerance * 70);
  EXPECT_GT(volume, 21.6);
  Obb zero_obb = Obb(RigidTransformd(Vector3d(3, -4, 1.3)), Vector3d(0, 0, 0));
  // Since a, b and c are 0, only the ε³ term is left and kTolerance³ is
  // within kTolerance.
  const double zero_volume = zero_obb.CalcVolume();
  EXPECT_NEAR(zero_volume, 0, kTolerance);
  EXPECT_GT(zero_volume, 0);
}

// Tests calculating the bounding points.
TEST_F(ObbTester, TestBounds) {
  // The box frame B is posed in the hierarchy frame H.
  const RigidTransformd X_HB(
      RotationMatrixd(RollPitchYawd(M_PI_4, M_PI / 3., M_PI_2)),
      Vector3d(-1, 2, 1));
  const Vector3d half_width(2, 0.5, 2.7);
  const Obb obb = Obb(X_HB, half_width);
  // Padding (see PadBoundary()) introduced additional tolerance terms.
  const Vector3d tolerance = Vector3d::Constant(kTolerance);
  const Vector3d expect_upper = X_HB * (half_width + tolerance);
  const Vector3d expect_lower = X_HB * (-half_width - tolerance);
  EXPECT_TRUE(CompareMatrices(obb.upper(), expect_upper,
                              std::numeric_limits<double>::epsilon()));
  EXPECT_TRUE(CompareMatrices(obb.lower(), expect_lower,
                              std::numeric_limits<double>::epsilon()));
}

// Tests padding the boundary of the bounding box volume.
TEST_F(ObbTester, TestPadBoundary) {
  Obb obb = Obb(RigidTransformd(Vector3d(-1, 0.5, 1)), Vector3d(1.2, 2.5, 0.3));
  Vector3d padded = Vector3d(1.2, 2.5, 0.3).array() + kTolerance;
  EXPECT_TRUE(CompareMatrices(obb.half_width(), padded));

  // Large boxes should have a bigger padding based on either the maximum
  // half width or position in the frame.
  const double padding = 300 * std::numeric_limits<double>::epsilon();
  ASSERT_GT(padding, kTolerance);
  // Max is set from half_width.z.
  obb = Obb(RigidTransformd(Vector3d(-1, 1.5, 1)), Vector3d(120, 250, 300));
  padded = Vector3d(120, 250, 300).array() + padding;
  // Expect the two Vector3d to be exactly equal down to the last bit.
  EXPECT_TRUE(CompareMatrices(obb.half_width(), padded));
  // Max is set from |center.x|.
  obb = Obb(RigidTransformd(Vector3d(-300, 50, 100)), Vector3d(1, 2, 0.5));
  padded = Vector3d(1, 2, 0.5).array() + padding;
  // Expect the two Vector3d to be exactly equal down to the last bit.
  EXPECT_TRUE(CompareMatrices(obb.half_width(), padded));
}

// We want to compute X_AB such that B is posed relative to A as documented in
// TestObbOverlap. We can do so by generating the rotation component, R_AB, such
// that Bq has a minimum value along the chosen axis, and we can solve for
// the translation component, p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A.
RigidTransformd CalcCornerTransform(const Obb& a, const Obb& b, const int axis,
                                    const bool expect_overlap) {
  const int axis1 = (axis + 1) % 3;
  const int axis2 = (axis + 2) % 3;
  // Construct the rotation matrix, R_AB, that has meaningful (non-zero)
  // values everywhere for the remaining 2 axes and no symmetry.
  const RotationMatrixd R_AB =
      RotationMatrixd(AngleAxisd(M_PI / 5, Vector3d::Unit(axis1))) *
      RotationMatrixd(AngleAxisd(-M_PI / 5, Vector3d::Unit(axis2)));

  // We define p_BqBo in Frame A from box b's minimum corner to its center.
  const Vector3d p_BqBo_A = R_AB * b.half_width();
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
  Vector3d p_AoAf_A = a.half_width();
  p_AoAf_A[axis1] /= 2;
  p_AoAf_A[axis2] /= 2;

  const Vector3d p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A;
  return RigidTransformd(R_AB, p_AoBo_A);
}

// We want to compute X_AB such that B is posed relative to A as documented
// in TestObbOverlap. We can do so by generating the rotation component, R_AB,
// such that Bq lies on the minimum edge along the chosen axis, and we can solve
// for the translation component, p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A.
RigidTransformd CalcEdgeTransform(const Obb& a, const Obb& b, const int a_axis,
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
  Vector3d p_BqBo_B = b.half_width();
  p_BqBo_B[b_axis] -= b.half_width()[b_axis] / 2;
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
  Vector3d p_AoAf_A = a.half_width();
  p_AoAf_A[a_axis] -= a.half_width()[a_axis] / 2;

  Vector3d p_AoBo_A = p_AoAf_A + p_AfBq_A + p_BqBo_A;
  // Finally we combine the components to form the transform X_AB.
  return RigidTransformd(R_AB, p_AoBo_A);
}

// Tests whether OBBs overlap. There are 15 cases to test, each covering a
// separating axis between the two bounding boxes. The first 3 cases use the
// axes of Frame A, the next 3 cases use the axes of Frame B, and the remaining
// 9 cases use the axes defined by the cross product of axes from Frame A and
// Frame B. We also test that it is robust for the case of parallel boxes.
GTEST_TEST(ObbTest, TestObbOverlap) {
  // Frame A of box `a` is arbitrarily posed in the hierarchy frame G.
  const RigidTransformd X_GA{
      RotationMatrixd(RollPitchYawd(2. * M_PI / 3., M_PI_4, -M_PI / 3.)),
      Vector3d(1, 2, 3)};
  // Frame B of box `b` is arbitrarily posed in the hierarchy frame H.
  const RigidTransformd X_HB{
      RotationMatrixd(RollPitchYawd(M_PI_4, M_PI / 5., M_PI / 6.)),
      Vector3d(2, 0.5, 4)};
  const RigidTransformd X_BH = X_HB.inverse();

  // One box is fully contained in the other and they are parallel.
  Obb a = Obb(X_GA, Vector3d(1, 2, 1));
  Obb b = Obb(X_HB, Vector3d(0.5, 1, 0.5));
  RigidTransformd X_AB = RigidTransformd::Identity();
  EXPECT_TRUE(Obb::HasOverlap(a, b, X_GA * X_AB * X_BH /* X_GH */));

  // To cover the cases of the axes of Frame A, we need to pose box B along
  // each axis. For example, in the case of the Ax-axis, in a 2D view they would
  // look like:
  //           Ay
  //           ^
  //   --------|--------      ⋰ ⋱       ↗By
  //   |       |       |   ⋰      ⋱  ↗
  //   |       |       Af Bq       ↗⋱
  //   |       |       |     ⋱   Bo   ⋱
  //   |      Ao--------->Ax   ⋱    ↘   ⋱
  //   |               |         ⋱    ↘⋰
  //   |               |           ⋱⋰   ↘Bx
  //   |               |
  //   -----------------
  //
  //                                Hy
  //                                ⇑
  // Gy             Gx              ⇑
  //   ⇖           ⇗                ⇑
  //     ⇖       ⇗                  ⇑
  //       ⇖   ⇗                    ⇑
  //         Go                     Ho ⇒ ⇒ ⇒ ⇒ ⇒ Hx
  //
  //
  // For this test, we define Point Bq as the minimum corner of the box B (i.e.,
  // center - half width). We want to pose box B so Bq is the uniquely closest
  // point to box A at a Point Af in the interior of +Ax face. The rest of
  // the box B extends farther along the +Ax axis (as suggested in the above
  // illustration). Point Bq will be a small epsilon away from the nearby face
  // either outside (if expect_overlap is false) or inside (if true).
  a = Obb(X_GA, Vector3d(2, 4, 3));
  b = Obb(X_HB, Vector3d(3.5, 2, 1.5));
  for (int axis = 0; axis < 3; ++axis) {
    X_AB = CalcCornerTransform(a, b, axis, false /* expect_overlap */);
    EXPECT_FALSE(Obb::HasOverlap(a, b, X_GA * X_AB * X_BH /* X_GH */));
    X_AB = CalcCornerTransform(a, b, axis, true /* expect_overlap */);
    EXPECT_TRUE(Obb::HasOverlap(a, b, X_GA * X_AB * X_BH /* X_GH */));
  }

  // To cover the local axes out of B, we can use the same method by swapping
  // the order of the boxes and then using the inverse of the transform.
  for (int axis = 0; axis < 3; ++axis) {
    X_AB =
        CalcCornerTransform(b, a, axis, false /* expect_overlap */).inverse();
    EXPECT_FALSE(Obb::HasOverlap(a, b, X_GA * X_AB * X_BH /* X_GH */));
    X_AB = CalcCornerTransform(b, a, axis, true /* expect_overlap */).inverse();
    EXPECT_TRUE(Obb::HasOverlap(a, b, X_GA * X_AB * X_BH /* X_GH */));
  }

  // To cover the remaining 9 cases, we need to pose an edge from box B along
  // an edge from box A. The axes that the edges are aligned with form the
  // two inputs into the cross product for the separating axis. For example,
  // in the following illustration, Af lies on the edge aligned with A's y-axis.
  // Assuming that Bq lies on an edge aligned with B's x-axis, this would form
  // the case testing the separating axis Ay × Bx.
  //                       _________
  //   +z                 /________/\              .
  //    ^                 \        \ \             .
  //    |   ______________ Bq       \ \            .
  //    |  |\             Af \  Bo   \ \           .
  //    |  | \ _____________\ \       \ \          .
  // +y |  | |      Ao      |  \_______\/          .
  //  \ |  \ |              |                      .
  //   \|   \|______________|                      .
  //    -----------------------------------> +x
  //
  // For this test, we define point Bq on the minimum edge of the box in its
  // own frame (i.e., center - half width + an offset along the edge). We want
  // to pose box B so Bq is the uniquely closest point to A at a Point Af on the
  // edge between the +x and +z face of box A. The rest of box B extends
  // farther along the +Ax and +Az axis (as suggested in the above
  // illustration). Point Bq will be a small epsilon away from the nearby
  // edge either outside (if expect_overlap is false) or inside (if true).
  for (int a_axis = 0; a_axis < 3; ++a_axis) {
    for (int b_axis = 0; b_axis < 3; ++b_axis) {
      X_AB =
          CalcEdgeTransform(a, b, a_axis, b_axis, false /* expect_overlap */);
      // Separate along a's y-axis and b's x-axis.
      EXPECT_FALSE(Obb::HasOverlap(a, b, X_GA * X_AB * X_BH  /* X_GH */));
      X_AB =
          CalcEdgeTransform(a, b, a_axis, b_axis, true /* expect_overlap */);
      // Separate along a's y-axis and b's x-axis.
      EXPECT_TRUE(Obb::HasOverlap(a, b, X_GA * X_AB * X_BH  /* X_GH */));
    }
  }
}

// Tests Obb-plane intersection. We have four frames:
//
//   B: the canonical frame the box is defined in (centered on Bo and aligned
//      with B's axes).
//   H: the hierarchy frame in which the box frame B is posed.
//   P: the frame the plane is defined in.
//   Q: the frame the query is performed in.
//
// For simplicity, we'll define the plane with a normal in the Pz direction
// passing through Po. We'll pose the box relative to the plane (so we can
// easily reason about whether it penetrates or not). But then express the plane
// in the query frame Q (and the pose of B in Q).
GTEST_TEST(ObbTest, PlaneOverlap) {
  // The obb is *not* defined at the origin of the hierarchy frame.
  const Vector3d p_HoBo_H = Vector3d{0.5, 0.25, -0.75};
  const Obb obb_H{
      RigidTransformd{RollPitchYawd(2. * M_PI / 3., M_PI_4, -M_PI / 3.),
                      p_HoBo_H},
      Vector3d{1, 2, 3}};

  // Use brute force to find the position of the "lowest" corner of the box
  // measured from Ho and expressed in frame P. "Lowest" means the corner with
  // the smallest z-component. Note: the "z-component" trick only works because
  // we expect the plane to be Pz = 0.
  auto lowest_corner = [&obb_H](const RotationMatrixd& R_PH) {
    Vector3d p_HoCmin_P =
        Vector3d::Constant(std::numeric_limits<double>::infinity());
    for (const double x_sign : {-1.0, 1.0}) {
      for (const double y_sign : {-1.0, 1.0}) {
        for (const double z_sign : {-1.0, 1.0}) {
          const Vector3d signs{x_sign, y_sign, z_sign};
          const Vector3d p_HoC_H =
              obb_H.pose() * obb_H.half_width().cwiseProduct(signs);
          const Vector3d p_HoC_P = R_PH * p_HoC_H;
          if (p_HoC_P(2) < p_HoCmin_P(2)) {
            p_HoCmin_P = p_HoC_P;
          }
        }
      }
    }
    return p_HoCmin_P;
  };

  // Test epsilon is the product of three factors:
  //  - machine epsilon
  //  - Two orders of magnitude attributed to the various transformations.
  //  - A scale factor that is the maximum of (box size, p_HoBo, p_PoHo)
  const double kEps = 300 * std::numeric_limits<double>::epsilon();
  // An arbitrary collection of orientations for the box's hierarchy frame H
  // in the plane frame P.
  std::vector<AngleAxisd> R_PHs{
      AngleAxisd{0, Vector3d::UnitX()},
      AngleAxisd{M_PI / 2, Vector3d::UnitX()},
      AngleAxisd{M_PI / 2, Vector3d::UnitY()},
      AngleAxisd{M_PI / 2, Vector3d::UnitZ()},
      AngleAxisd{M_PI / 4, Vector3d::UnitX()},
      AngleAxisd{M_PI / 4, Vector3d::UnitY()},
      AngleAxisd{M_PI / 7, Vector3d{1, 2, 3}.normalized()},
      AngleAxisd{7 * M_PI / 6, Vector3d{-1, 2, -3}.normalized()},
      AngleAxisd{12 * M_PI / 7, Vector3d{1, -2, 3}.normalized()}
  };
  // An arbitrary collection of poses of the plane in the query frame Q.
  std::vector<RigidTransformd> X_QPs{
      RigidTransformd{},  // Identity matrix.
      RigidTransformd{
          RotationMatrixd{AngleAxisd{M_PI / 4, Vector3d{1, 2, 3}.normalized()}},
          Vector3d{1, 2, 3}},
      RigidTransformd{RotationMatrixd{AngleAxisd{
                          12 * M_PI / 7, Vector3d{-1, -1, 3}.normalized()}},
                      Vector3d{-3, -1, 2}}};
  for (const auto& angle_axis_PH : R_PHs) {
    const RotationMatrixd R_PH{angle_axis_PH};
    const Vector3d p_HoCmin_P = lowest_corner(R_PH);
    for (const auto& X_QP : X_QPs) {
      // Define the plane in the query frame Q.
      const Vector3d& Pz_Q = X_QP.rotation().col(2);
      Plane<double> plane_Q{Pz_Q, X_QP.translation()};

      // We position Ho such that Cmin lies on the z = 0 plane in Frame P. Given
      // we know p_HoCmin_P, we know its current z-value. To put it at zero, we
      // must displace it in the negative of that z value. The x- and y-values
      // don't matter, so we pick values we know not to be zero.
      {
        // Place the minimum corner just "above" the plane.
        const Vector3d p_PoHo_P{Vector3d{0.5, -0.25, -p_HoCmin_P(2) + kEps}};
        RigidTransformd X_PH{R_PH, p_PoHo_P};
        EXPECT_FALSE(Obb::HasOverlap(obb_H, plane_Q, X_QP * X_PH));
      }
      {
        // Place the minimum corner just "below" the plane.
        const Vector3d p_PoHo_P{Vector3d{0.5, -0.25, -p_HoCmin_P(2) - kEps}};
        RigidTransformd X_PH{R_PH, p_PoHo_P};
        EXPECT_TRUE(Obb::HasOverlap(obb_H, plane_Q, X_QP * X_PH));
      }

      // We repeat the same task but with Cmax. Cmax is the reflection of Cmin
      // over Bo (the origin of the box). We'll express all vectors in the P
      // frame so we can place that corner just above and below the Pz = 0
      // plane using the same trick as documented above.
      const Vector3d p_HoBo_P = R_PH * p_HoBo_H;
      const Vector3d p_HoCmax_P = p_HoCmin_P + 2 * (p_HoBo_P - p_HoCmin_P);
      {
        // Put the maximum corner *on* the z = 0 plane in Frame P. The bulk of
        // the box now extends *below* the plane; so bump it up epsilon to
        // guarantee intersection.
        const Vector3d p_PoHo_P{Vector3d{0.5, -0.25, -p_HoCmax_P(2) + kEps}};
        RigidTransformd X_PH{R_PH, p_PoHo_P};
        EXPECT_TRUE(Obb::HasOverlap(obb_H, plane_Q, X_QP * X_PH));
      }
      {
        // Put the maximum corner *on* the z = 0 plane in Frame P. The bulk of
        // the box now extends *below* the plane; so bump it down epsilon to
        // guarantee _no_ intersection.
        const Vector3d p_PoHo_P{Vector3d{0.5, -0.25, -p_HoCmax_P(2) - kEps}};
        RigidTransformd X_PH{R_PH, p_PoHo_P};
        EXPECT_FALSE(Obb::HasOverlap(obb_H, plane_Q, X_QP * X_PH));
      }
    }
  }
}


// Tests the determination of an Obb intersects a half space.
GTEST_TEST(ObbTest, HalfSpaceOverlap) {
  // We'll rely on the pose to reposition the box.
  const Vector3d p_HoBo_H = Vector3d{0.25, -0.5, 0.75};
  const Obb obb_H{
      RigidTransformd{RollPitchYawd(2. * M_PI / 3., M_PI_4, -M_PI / 3.),
                      p_HoBo_H},
      Vector3d{1, 2, 3}};

  // Find the "lowest" corner of the obb relative to the half space. That means
  // the corner that has the smallest "z" value when expressed in Frame C.
  // We return the corner measured in the H frame and expressed in the C frame.
  // We use a brute force method to distinguish from the "cleverness" in the
  // Obb algorithm.
  auto lowest_corner = [&obb_H](const RotationMatrixd& R_CH) {
    Vector3d p_HoVmin_C =
        Vector3d::Constant(std::numeric_limits<double>::infinity());
    for (const double x_sign : {-1.0, 1.0}) {
      for (const double y_sign : {-1.0, 1.0}) {
        for (const double z_sign : {-1.0, 1.0}) {
          const Vector3d signs{x_sign, y_sign, z_sign};
          const Vector3d p_HoV_H =
              obb_H.pose() * obb_H.half_width().cwiseProduct(signs);
          const Vector3d p_HoV_C = R_CH * p_HoV_H;
          if (p_HoV_C(2) < p_HoVmin_C(2)) {
            p_HoVmin_C = p_HoV_C;
          }
        }
      }
    }
    return p_HoVmin_C;
  };

  const double kEps = 100 * std::numeric_limits<double>::epsilon();
  // An arbitrary collection of orientations.
  std::vector<AngleAxisd> R_CHs{
      AngleAxisd{0, Vector3d::UnitX()},
      AngleAxisd{M_PI / 2, Vector3d::UnitX()},
      AngleAxisd{M_PI / 2, Vector3d::UnitY()},
      AngleAxisd{M_PI / 2, Vector3d::UnitZ()},
      AngleAxisd{M_PI / 4, Vector3d::UnitX()},
      AngleAxisd{M_PI / 4, Vector3d::UnitY()},
      AngleAxisd{M_PI / 7, Vector3d{1, 2, 3}.normalized()},
      AngleAxisd{7 * M_PI / 6, Vector3d{-1, 2, -3}.normalized()},
      AngleAxisd{12 * M_PI / 7, Vector3d{1, -2, 3}.normalized()}
  };
  const HalfSpace hs_C;

  for (const auto& angle_axis_CH : R_CHs) {
    const RotationMatrixd R_CH{angle_axis_CH};
    const Vector3d p_HoVmin_C = lowest_corner(R_CH);
    // We position Ho such that Vmin lies on the z = 0 plane in Frame C. Given
    // we know p_HoVmin_C, we know its current z-value. To put it at zero, we
    // must displace it in the negative of that z value. The x- and y-values
    // don't matter, so we pick values we know not to be zero.
    {
      // Place the minimum corner just "outside" the half space.
      // Note: It's unclear why this particular test requires an epsilon twice
      // as large (compared to the other tests) to work across all
      // configurations of R_CH.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmin_C(2) + 2 * kEps}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_FALSE(Obb::HasOverlap(obb_H, hs_C, X_CH));
    }
    {
      // Place the minimum corner just "below" the plane.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmin_C(2) - kEps}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_TRUE(Obb::HasOverlap(obb_H, hs_C, X_CH));
    }

    // As soon as the box penetrates the half space, no amount of movement
    // against the surface normal direction will ever report a non-overlapping
    // state. We sample from that sub-domain by pushing the maximum corner
    // (Vmax) near the boundary and then push the whole box deep into the
    // half space.
    //
    // Vmax is the reflection of Vmin over Bo (the origin of the box). We'll
    // express all vectors in the C frame so we can place that corner just above
    // and below the z = 0 plane in Frame C using the same trick as documented
    // above.
    const Vector3d p_HoBo_C = R_CH * p_HoBo_H;
    const Vector3d p_HoVmax_C = p_HoVmin_C + 2 * (p_HoBo_C - p_HoVmin_C);
    {
      // Put the maximum corner just above the z = 0 plane in Frame C.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmax_C(2) + kEps}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_TRUE(Obb::HasOverlap(obb_H, hs_C, X_CH));
    }

    {
      // Put the maximum corner just below the z = 0 plane in Frame C.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmax_C(2) - kEps}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_TRUE(Obb::HasOverlap(obb_H, hs_C, X_CH));
    }

    {
      // Bury the box deep in the half space.
      const Vector3d p_CoHo_C{Vector3d{0.5, -0.25, -p_HoVmax_C(2) - 1e8}};
      RigidTransformd X_CH{R_CH, p_CoHo_C};
      EXPECT_TRUE(Obb::HasOverlap(obb_H, hs_C, X_CH));
    }
  }
}

GTEST_TEST(ObbTest, TestEqual) {
  Obb a{RigidTransformd(Vector3d{0.5, 0.25, -0.75}), Vector3d{1, 2, 3}};
  // Equal to itself.
  EXPECT_TRUE(a.Equal(a));
  // Different pose.
  Obb b{RigidTransformd(Vector3d{1.5, 1.25, -1.75}), Vector3d{1, 2, 3}};
  EXPECT_FALSE(a.Equal(b));
  // Different half_width.
  Obb c{RigidTransformd(Vector3d{0.5, 0.25, -0.75}), Vector3d{2, 4, 6}};
  EXPECT_FALSE(a.Equal(c));
  // Same.
  Obb d{RigidTransformd(Vector3d{0.5, 0.25, -0.75}), Vector3d{1, 2, 3}};
  EXPECT_TRUE(a.Equal(d));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
