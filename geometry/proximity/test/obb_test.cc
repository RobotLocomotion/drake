#include "drake/geometry/proximity/obb.h"

#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_convex_hull_mesh.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

// TODO(SeanCurtis-TRI): Add test coverage for obb makers for AutoDiffXd typed
// meshes.
// Friend class for accessing Obb's private functionality.
class ObbTester : public ::testing::Test {
 public:
  static constexpr double kTolerance = Obb::kTolerance;
};

// Friend class for accessing ObbMaker's private functionality for testing.
template <typename MeshType>
class ObbMakerTester {
 public:
  ObbMakerTester(const MeshType& mesh_M, const std::set<int>& vertices)
      : obb_maker_(mesh_M, vertices) {}

  RotationMatrixd CalcOrientationByPca() const {
    return obb_maker_.CalcOrientationByPca();
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

// Verifies the order of eigenvalues from Eigen::SelfAdjointEigenSolver in
// all representative cases. We are verifying that an Eigen utility works as
// advertised (rather than testing some of our own code) because:
//
// 1. There is no standard choice of the order (increasing or decreasing) of
//    eigenvalues among different linear-algebra libraries. Eigen library
//    chooses the increasing order.
// 2. Our CalcOrientationByPca() promises to return principal components,
//    which, by definition:
//    https://en.wikipedia.org/wiki/Principal_component_analysis
//    must start from the most important component, i.e., the largest
//    dimension must be in Bx direction of the OBB's frame B, and the smallest
//    dimension must be in Bz direction.
// 3. Our code relies on the consistent order of eigenvalues from Eigen, so
//    we can fulfill the promise in 2. If an upgrade to Eigen library changes
//    it, we want to know right away.
//
// Additionally, the concepts of ranks of covariance matrices and multiplicities
// of their eigenvalues could be perplexing to engineers (including this
// author), and they can affect our applications. For example, multiplicity
// can cause an arbitrary choice of basis that leaves us with a poor OBB.
// Therefore, this unit test also demonstrates examples of these special cases.
//
// This table shows classification of the cases regarding ranks and
// multiplicities of eigenvalues of covariance matrices:
// -------+--------------------------
//        |       multiplicity
//        |    1       2       3
// -------+--------------------------
// rank 3 |  case 1  case 5  case 4
//      2 |  case 2  case 3
//      1 |          case 6
//      0 |                  case 7
// -------+--------------------------
//         multiplicity 1 = all 3 eigenvalues are distinct.
//         multiplicity 2 = two repeated eigenvalues + 1 unique eigenvalue.
//         multiplicity 3 = all 3 eigenvalues are the same.
//
// Examples:
// case 1: Vertices of a box of different sizes in each dimension.
// case 2: Vertices of a rectangle (width ≠ height).
// case 3: Vertices of an equilateral triangle or a square.
// case 4: Vertices of a regular octahedron (and possibly other Platonic
//         solids: tetrahedron, cube, dodecahedron, icosahedron).
// case 5: Take a regular octahedron and stretch (or shrink) it in one diagonal
//         direction.
// case 6: Collinear points. (Two zero eigenvalues + one positive eigenvalue.)
// case 7: One single point. (All three eigenvalues are zero.)
//
// However, we only test cases 1, 2, 3, 4, 5 because they are most relevant
// to us. In the future, if cases 6 and 7 become important, we can add them.
GTEST_TEST(ObbMakerTest, VerifyEigenvalues) {
  // Empirical tolerance from solving eigenvalue problems, about 2e-14. You
  // might have to change it if you change the examples.
  const double kEps = 100.0 * std::numeric_limits<double>::epsilon();

  // In all cases, points are measured and expressed in frame M,
  // and C is the centroid (average) of the points.

  // Case 1: (rank 3, multiplicity 1).
  // Covariance matrix of 8 vertices of the box [-1,-1]x[-2,2]x[-3,3]
  // has distinct eigenvalues in increasing order: 1, 2², 3².
  {
    Matrix3d covariance_M = Matrix3d::Zero();
    for (const double x : {-1., -1.}) {
      for (const double y : {-2., 2.}) {
        for (const double z : {-3., 3.}) {
          // The centroid C of the points is at Mo, so p_CV_M = p_MV = (x,y,z).
          const Vector3d p_CV_M(x, y, z);
          covariance_M += p_CV_M * p_CV_M.transpose();
        }
      }
    }
    covariance_M /= 8.;
    Eigen::SelfAdjointEigenSolver<Matrix3d> es;
    es.computeDirect(covariance_M);
    EXPECT_NEAR(es.eigenvalues()[0], 1., kEps);
    EXPECT_NEAR(es.eigenvalues()[1], 4., kEps);
    EXPECT_NEAR(es.eigenvalues()[2], 9., kEps);
  }

  // Case 2: (rank 2, multiplicity 1).
  // Covariance matrix of 4 vertices of the rectangle [-1,-1]x[-2,-2]x{0}
  // has distinct eigenvalues: 0, 1, 2². The least value is 0 because the
  // points are co-planar.
  {
    Matrix3d covariance_M = Matrix3d::Zero();
    for (const double x : {-1., -1.}) {
      for (const double y : {-2., 2.}) {
        // The centroid C of the points is at Mo, so p_CV_M = p_MV = (x,y,0).
        const Vector3d p_CV_M(x, y, 0);
        covariance_M += p_CV_M * p_CV_M.transpose();
      }
    }
    covariance_M /= 4.;
    Eigen::SelfAdjointEigenSolver<Matrix3d> es;
    es.computeDirect(covariance_M);
    EXPECT_NEAR(es.eigenvalues()[0], 0., kEps);
    EXPECT_NEAR(es.eigenvalues()[1], 1., kEps);
    EXPECT_NEAR(es.eigenvalues()[2], 4., kEps);
  }

  // Case 3: (rank 2, multiplicity 2).
  // Covariance matrix of 3 vertices of an equilateral triangle {UnitX(),
  // UnitY(), UnitZ()} has eigenvalues 0, 1/3, 1/3. The least value is 0
  // because the points are co-planar.
  {
    Matrix3d covariance_M = Matrix3d::Zero();
    Vector3d p_MC(1. / 3., 1. / 3., 1. / 3.);
    for (const auto& p_MV :
         {Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()}) {
      const Vector3d p_CV_M = p_MV - p_MC;
      covariance_M += p_CV_M * p_CV_M.transpose();
    }
    covariance_M /= 3.;
    Eigen::SelfAdjointEigenSolver<Matrix3d> es;
    es.computeDirect(covariance_M);
    EXPECT_NEAR(es.eigenvalues()[0], 0., kEps);
    EXPECT_NEAR(es.eigenvalues()[1], 1. / 3., kEps);
    EXPECT_NEAR(es.eigenvalues()[2], 1. / 3., kEps);
  }

  // Case 4: (rank 3, multiplicity 3).
  // Covariance matrix of 6 vertices of a "unit" regular octahedron has one
  // eigenvalue repeated three times: 2, 2, 2.
  {
    Matrix3d covariance_M = Matrix3d::Zero();
    // Use std::vector<> here to instruct compiler to convert every entry to
    // Vector3d. Strictly speaking, Vector3d and -Vector3d have different
    // types.
    for (const Vector3d& p_MV : std::vector<Vector3d>{
             Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ(),
             -Vector3d::UnitX(), -Vector3d::UnitY(), -Vector3d::UnitZ()}) {
      // The centroid C is at Mo, so we can use p_MV for p_CV_M.
      covariance_M += p_MV * p_MV.transpose();
    }
    covariance_M / 6.;
    Eigen::SelfAdjointEigenSolver<Matrix3d> es;
    es.computeDirect(covariance_M);
    EXPECT_NEAR(es.eigenvalues()[0], 2., kEps);
    EXPECT_NEAR(es.eigenvalues()[1], 2., kEps);
    EXPECT_NEAR(es.eigenvalues()[2], 2., kEps);
  }

  // Case 5: (rank 3, multiplicity 2).
  // Stretching the previous example 3X in Mx direction, its covariance matrix
  // has one eigenvalue repeated two times followed by the largest
  // eigenvalue: 2, 2, 2²*2.
  {
    Matrix3d covariance_M = Matrix3d::Zero();
    // Use std::vector<> here to instruct compiler to convert every entry to
    // Vector3d. Strictly speaking, Vector3d and -Vector3d have different
    // types.
    for (const Vector3d& p_MV : std::vector<Vector3d>{
             2. * Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ(),
             -2. * Vector3d::UnitX(), -Vector3d::UnitY(), -Vector3d::UnitZ()}) {
      // The centroid C is at Mo, so we can use p_MV for p_CV_M.
      covariance_M += p_MV * p_MV.transpose();
    }
    covariance_M / 6.;
    Eigen::SelfAdjointEigenSolver<Matrix3d> es;
    es.computeDirect(covariance_M);
    EXPECT_NEAR(es.eigenvalues()[0], 2., kEps);
    EXPECT_NEAR(es.eigenvalues()[1], 2., kEps);
    EXPECT_NEAR(es.eigenvalues()[2], 8., kEps);
  }
}

// Test fixture class for testing PCA with a unique solution up to sign flips,
// so it is invariant under rigid transform modulo sign-flips. We use a
// rectangular box that is not uniform (width < depth < height).
class ObbMakerTestRectangularBox : public ::testing::Test {
 public:
  ObbMakerTestRectangularBox()
      : ::testing::Test(),
        // Resolution hint 10 guarantees that the box mesh will be the
        // coarsest possible mesh with only 8 vertices and 12 triangles.
        mesh_M_(internal::MakeBoxSurfaceMesh<double>(Box(2, 4, 6), 10)) {}

  void SetUp() override {
    ASSERT_EQ(mesh_M_.num_vertices(), 8);
    for (int i = 0; i < mesh_M_.num_vertices(); ++i) {
      test_vertices_.insert(i);
    }
  }

 protected:
  TriangleSurfaceMesh<double> mesh_M_;
  std::set<int> test_vertices_;
};

// Test the creation of obb orientation via PCA. We want to test the following:
//   1. The resulting basis is right-handed.
//   2. The basis is, in some sense, invariant to rigid transformations.
//   3. The function will return a predictable basis.
TEST_F(ObbMakerTestRectangularBox, CalcOrientationByPca) {
  // B is the frame of PCA solution for the oriented bounding box. We will
  // check it against the tested box's frame M.
  const RotationMatrixd R_MB =
      ObbMakerTester<TriangleSurfaceMesh<double>>(mesh_M_, test_vertices_)
          .CalcOrientationByPca();
  const Vector3d Mx_M = Vector3d::UnitX();
  const Vector3d My_M = Vector3d::UnitY();
  const Vector3d Mz_M = Vector3d::UnitZ();
  const Vector3d& Bx_M = R_MB.col(0);
  const Vector3d& By_M = R_MB.col(1);
  const Vector3d& Bz_M = R_MB.col(2);
  // Empirical tolerance for comparing unit vectors from characteristic
  // equation AX = λX and rigid transform.
  double kEps = 100. * std::numeric_limits<double>::epsilon();
  // Simple analysis:
  // 1. The box of size 2x4x6 has the half-width vector (1,2,3).
  // 2. The covariance matrix of its six vertices has distinct
  //    eigenvalues 1, 2², 3².
  // 3. PCA solution is unique up to sign flips.
  // 4. The principal directions Bx, By, Bz are along Mz, My, Mx of the
  // tested box because PCA directions are sorted in decreasing order of size
  // (the box is longest in its Mz direction).
  //
  // Both principal direction D and -D are equivalent in PCA, so we allow two
  // possibilities in each check below. (If V is an eigenvector of AX=λX, so
  // is -V.)
  //
  // We use the box dimensions in Mx,My,Mz in increasing order, but
  // the principal components are always in the order of decreasing variance.
  EXPECT_TRUE(CompareMatrices(Bx_M, Mz_M, kEps) ||
              CompareMatrices(Bx_M, -Mz_M, kEps));
  EXPECT_TRUE(CompareMatrices(By_M, My_M, kEps) ||
              CompareMatrices(By_M, -My_M, kEps));
  EXPECT_TRUE(CompareMatrices(Bz_M, Mx_M, kEps) ||
              CompareMatrices(Bz_M, -Mx_M, kEps));
  // Confirm the basis is right-handed.
  EXPECT_TRUE(R_MB.IsValid());

  // Apply rigid transform X_FM, and test that the PCA solution is invariant
  // under rigid transform (modulo sign flips) from frame M to frame F. We can
  // do this because this example's PCA solution is unique up to sign flips.
  const RotationMatrixd R_FM(
      RollPitchYawd(2. * M_PI / 3., M_PI / 4., M_PI / 5.));
  const RigidTransformd X_FM(R_FM, Vector3d(-0.2, 2, 1.5));
  mesh_M_.TransformVertices(X_FM);
  // Now we use alias mesh_F because `mesh_M_` has been transformed; its stored
  // vertices are now measured and expressed in frame F.
  const TriangleSurfaceMesh<double>& mesh_F = mesh_M_;

  const RotationMatrixd R_FB =
      ObbMakerTester<TriangleSurfaceMesh<double>>(mesh_F, test_vertices_)
          .CalcOrientationByPca();
  const Vector3d& Bx_F = R_FB.col(0);
  const Vector3d& By_F = R_FB.col(1);
  const Vector3d& Bz_F = R_FB.col(2);
  // Check components of the new orientation R_FB. Allow sign flip for the
  // same reason as before.
  EXPECT_TRUE(CompareMatrices(Bx_F, R_FM * Bx_M, kEps) ||
              CompareMatrices(Bx_F, -(R_FM * Bx_M), kEps));
  EXPECT_TRUE(CompareMatrices(By_F, R_FM * By_M, kEps) ||
              CompareMatrices(By_F, -(R_FM * By_M), kEps));
  EXPECT_TRUE(CompareMatrices(Bz_F, R_FM * Bz_M, kEps) ||
              CompareMatrices(Bz_F, -(R_FM * Bz_M), kEps));
  // Confirm the basis is right-handed.
  EXPECT_TRUE(R_FB.IsValid());
}

// The purpose of this test fixture is:
// 1. Ensures test coverage of ObbMaker::CalcOrientationByPca() and
//    ObbMaker::CalcOrientedBox().
// 2. Tests an example with non-unique PCA solution with eigenvalue of
//    multiplicity 2.
//
// We use an equilateral triangle whose vertices are at unit distance from the
// origin along each axis of frame M like this:
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
        mesh_M_({{0, 1, 2}},
                {Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()}),
        test_vertices_{0, 1, 2} {}

 protected:
  TriangleSurfaceMesh<double> mesh_M_;
  const std::set<int> test_vertices_;
};

TEST_F(ObbMakerTestTriangle, CalcOrientationByPca) {
  // B is the frame of PCA solution for the oriented bounding box. We will check
  // it against the mesh's frame M.
  const RotationMatrixd R_MB =
      ObbMakerTester<TriangleSurfaceMesh<double>>(mesh_M_, test_vertices_)
          .CalcOrientationByPca();
  const Vector3d Bx_M = R_MB.col(0);
  const Vector3d By_M = R_MB.col(1);
  const Vector3d Bz_M = R_MB.col(2);
  // Simple analysis of this example:
  // 1. Covariance matrix has eigenvalues 0, 1/3, 1/3.
  // 2. The largest eigenvalue 1/3 with multiplicity 2 indicates that the first
  //    two principal components span the plane of the triangle without a
  //    preferred line of direction. PCA solution is arbitrary; it can be any
  //    two basis vectors of the plane of the triangle.
  // 3. The smallest eigenvalue 0 means there is no variation left in the last
  //    principal component. It makes sense because there is no other points
  //    outside the plane of the triangle.
  //
  // We can only expect that Bx and By span the plane of triangle without
  // being more specific. As a result, we only check that Bz is ±N, where N
  // is the normal vector of the triangle.
  const Vector3d N_M = Vector3d(1, 1, 1).normalized();
  // Empirical tolerance for comparing unit vectors from characteristic
  // equation AX = λX and rigid transform.
  double kEps = 100. * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(Bz_M, N_M, kEps) ||
              CompareMatrices(Bz_M, -N_M, kEps));

  // Apply rigid transform X_FM, and show that the particular PCA solution that
  // we have is **not** invariant under rigid transform from frame M to frame F.
  // This is because the PCA solution is not unique.
  const RotationMatrixd R_FM(
      RollPitchYawd(2. * M_PI / 3., M_PI / 4., M_PI / 5.));
  const RigidTransformd X_FM(R_FM, Vector3d(-0.2, 2, 1.5));
  mesh_M_.TransformVertices(X_FM);
  // Now we use alias mesh_F because `mesh_M_` has been transformed; its stored
  // vertices are now measured and expressed in frame F.
  const TriangleSurfaceMesh<double>& mesh_F = mesh_M_;

  const RotationMatrixd R_FB =
      ObbMakerTester<TriangleSurfaceMesh<double>>(mesh_F, test_vertices_)
          .CalcOrientationByPca();
  const Vector3d& Bx_F = R_FB.col(0);
  const Vector3d& By_F = R_FB.col(1);
  const Vector3d& Bz_F = R_FB.col(2);
  // Check that Bz_F is the transform of Bz_M. It's the only unique
  // principal component (eigenvalue 0 without multiplicity).
  EXPECT_TRUE(CompareMatrices(Bz_F, R_FM * Bz_M, kEps) ||
              CompareMatrices(Bz_F, -(R_FM * Bz_M), kEps));
  // The other two principal components are not unique. In our current
  // implementation, we do not get the invariance under rigid transform. If
  // we change PCA implementation, we might have to change these checks.
  EXPECT_FALSE(CompareMatrices(Bx_F, R_FM * Bx_M, kEps) ||
               CompareMatrices(Bx_F, -(R_FM * Bx_M), kEps));
  EXPECT_FALSE(CompareMatrices(By_F, R_FM * By_M, kEps) ||
               CompareMatrices(By_F, -(R_FM * By_M), kEps));
}

TEST_F(ObbMakerTestTriangle, CalcOrientedBox) {
  // clang-format off
  const RotationMatrixd R_MB =
      RotationMatrixd::MakeFromOrthonormalColumns(
          Vector3d(1., -0.5, -0.5).normalized(),
          Vector3d(0., 1., -1.).normalized(),
          Vector3d(1., 1., 1.).normalized());
  // clang-format on

  const Obb obb =
      ObbMakerTester<TriangleSurfaceMesh<double>>(mesh_M_, test_vertices_)
          .CalcOrientedBox(R_MB);

  // Check the input R_MB passes to the output obb exactly.
  EXPECT_TRUE(obb.pose().rotation().IsExactlyEqualTo(R_MB));

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
             const std::set<int>& vertices) {
  const RigidTransformd& X_MB = obb_M.pose();
  const RigidTransformd X_BM = X_MB.inverse();
  for (int v : vertices) {
    Vector3d p_MV = mesh_M.vertex(v);
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
  // a general pose for testing robustness.

  // The ellipsoid's canonical frame is E, and the general frame is M.
  const RigidTransformd X_ME(
      RotationMatrixd(RollPitchYawd(2. * M_PI / 3., M_PI / 4., M_PI / 5.)),
      Vector3d(-0.2, 2, 1.5));
  // The first line calls it mesh_M even though it's actually mesh_E. The
  // second line correctly makes it mesh_M.
  TriangleSurfaceMesh<double> mesh_M =
      internal::MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1., 2., 3.), 6);
  mesh_M.TransformVertices(X_ME);
  // Confirm that it is an octahedron.
  ASSERT_EQ(8, mesh_M.num_triangles());
  ASSERT_EQ(6, mesh_M.num_vertices());
  const std::set<int> test_vertices{0, 1, 2, 3, 4, 5};

  // Initial obb is aligned with the three axes of the ellipsoid.
  const Obb initial_obb_M(X_ME, Vector3d(1., 2., 3.));
  const double initial_volume = initial_obb_M.CalcVolume();
  // The constructor Obb() already did padding for us. No need to pass
  // tolerance for numerics.
  ASSERT_TRUE(Contain(initial_obb_M, mesh_M, test_vertices));

  const Obb optimized_obb_M =
      ObbMakerTester<TriangleSurfaceMesh<double>>(mesh_M, test_vertices)
          .OptimizeObbVolume(initial_obb_M);
  EXPECT_TRUE(Contain(optimized_obb_M, mesh_M, test_vertices));
  const double percent_improvement =
      100. * (initial_volume - optimized_obb_M.CalcVolume()) / initial_volume;
  // Empirically we saw about 22% improvement for this particular example.
  // This check make sure the future code will not degrade this optimization.
  EXPECT_GT(percent_improvement, 22.0);

  // Now repeat the test, but with a mesh that is so small that there's no point
  // in optimizing it. The resulting "optimized" Obb should be identical to the
  // original Obb.

  // The whole mesh is much smaller than 1mm -- that should be sufficient.
  TriangleSurfaceMesh<double> mesh2_M =
      internal::MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1e-5, 2e-5, 3e-5),
                                                 6);
  mesh2_M.TransformVertices(X_ME);
  const Obb initial_obb2_M(X_ME, Vector3d(1e-5, 2e-5, 3e-5));
  const Obb optimized_obb2_M =
      ObbMakerTester<TriangleSurfaceMesh<double>>(mesh2_M, test_vertices)
          .OptimizeObbVolume(initial_obb2_M);
  // The pose and half_width will be bit identical as evidence that no work was
  // done when the volume would not change appreciably.
  EXPECT_TRUE(CompareMatrices(initial_obb2_M.half_width(),
                              optimized_obb2_M.half_width()));
  EXPECT_TRUE(CompareMatrices(initial_obb2_M.pose().GetAsMatrix34(),
                              optimized_obb2_M.pose().GetAsMatrix34()));
}

// TODO(DamrongGuoy): Update the tests when we improve the algorithm
//  for computing oriented bounding boxes.

// Test fixture for another corner case to show surprising results
// when a point set has its covariance matrix with triple multiplicity.
// Not only the OBB orientation from PCA can be arbitrary in all three
// directions, but also the post-optimization OBBs can be very different too.
// We use the vertex set of a regular octahedron. There are two special
// behaviors to demonstrate using this test fixture:
//   1. CalcOrientationByPca() can give arbitrary orientations in all three
//      directions under a rigid transform.
//   2. ObbMaker.Compute() can give very different volumes of oriented bounding
//      boxes under rigid transform, i.e., our algorithm is not always optimal.
class ObbMakerTestOctahedron : public ::testing::Test {
 public:
  ObbMakerTestOctahedron()
      : ::testing::Test(),
        // Use a coarse sphere, i.e. an octahedron, as the underlying mesh.
        mesh_M_(internal::MakeSphereSurfaceMesh<double>(Sphere(1.5), 3)),
        test_vertices_{0, 1, 2, 3, 4, 5} {}

  void SetUp() override { ASSERT_EQ(mesh_M_.num_vertices(), 6); }

 protected:
  TriangleSurfaceMesh<double> mesh_M_;
  const std::set<int> test_vertices_;
};

// Test PCA problem whose eigenvalue has triple multiplicity.
TEST_F(ObbMakerTestOctahedron, CalcOrientationByPca) {
  // B is the frame of PCA solution for the oriented bounding box. We will check
  // it against the mesh's frame M.
  const RotationMatrixd R_MB =
      ObbMakerTester<TriangleSurfaceMesh<double>>(mesh_M_, test_vertices_)
          .CalcOrientationByPca();
  const Vector3d Bx_M = R_MB.col(0);
  const Vector3d By_M = R_MB.col(1);
  const Vector3d Bz_M = R_MB.col(2);
  // Analysis of this example:
  // 1. Covariance matrix has eigenvalue 0.75 with multiplicity 3
  //    (det(A-λI) = (0.75-λ)³ = 0), so PCA solution is arbitrary.
  // 2. Mathematically the solution can be any orthonormal basis of ℝ³. The
  //    octahedron has no preferred direction measured by sum of squared
  //    distance.
  // 3. The result from CalcOrientationByPca() is deterministic but likely
  //    unstable under rigid transform.
  //
  // We will apply a rigid transform from frame M to frame F below and show that
  // we will get inconsistent PCA solution.
  const RotationMatrixd R_FM(
      RollPitchYawd(2. * M_PI / 3., M_PI / 4., M_PI / 5.));
  const RigidTransformd X_FM(R_FM, Vector3d(-0.2, 2, 1.5));
  mesh_M_.TransformVertices(X_FM);
  // Now we use alias mesh_F because `mesh_M_` has been transformed; its stored
  // vertices are now measured and expressed in frame F.
  const TriangleSurfaceMesh<double>& mesh_F = mesh_M_;

  const RotationMatrixd R_FB =
      ObbMakerTester<TriangleSurfaceMesh<double>>(mesh_F, test_vertices_)
          .CalcOrientationByPca();
  const Vector3d& Bx_F = R_FB.col(0);
  const Vector3d& By_F = R_FB.col(1);
  const Vector3d& Bz_F = R_FB.col(2);
  // Empirical tolerance for comparing unit vectors from characteristic
  // equation AX = λX and rigid transform.
  double kEps = 100. * std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(CompareMatrices(Bx_F, R_FM * Bx_M, kEps) ||
               CompareMatrices(Bx_F, -(R_FM * Bx_M), kEps));
  EXPECT_FALSE(CompareMatrices(By_F, R_FM * By_M, kEps) ||
               CompareMatrices(By_F, -(R_FM * By_M), kEps));
  EXPECT_FALSE(CompareMatrices(Bz_F, R_FM * Bz_M, kEps) ||
               CompareMatrices(Bz_F, -(R_FM * Bz_M), kEps));
}

// Test computing oriented bounding boxes of an octahedron in a standard pose
// and in a general pose, and show that the two cases have very different
// volumes, i.e., our algorithm is not always optimal.
//
// It is not easy to predict the pose and size of the expected bounding box,
// so we check the result indirectly in two steps:
// 1. Check that the box contains all the specified vertices.
// 2. Check that the box has volume around an empirically determined threshold.
//
// We also show that our algorithm is not always optimal. We use the same
// octahedron at two different poses and get two boxes with very different
// sizes.
TEST_F(ObbMakerTestOctahedron, ObbMakerCompute) {
  const Obb obb_M = ObbMaker(mesh_M_, test_vertices_).Compute();
  EXPECT_TRUE(Contain(obb_M, mesh_M_, test_vertices_));
  // The threshold was set empirically . It is slightly smaller than the
  // trivial upper bound 27.0, which is the volume of a cube bounding the
  // sphere. It is possible to do better as shown in the next test below.
  EXPECT_NEAR(obb_M.CalcVolume(), 26.997, 0.001);

  // The covariance matrix of the test vertices has one single eigenvalue with
  // multiplicity 3, so mathematically the PCA solution is arbitrary; any
  // arbitrary orthonormal basis spanning ℝ³ is a valid solution.
  // Computationally, the solution for this example is unstable. A rigid
  // transform from frame M to frame F below can perturb the new solution
  // significantly. As a result, the OBB's volume will shrink about 66%.
  const RigidTransformd X_MF(
      RotationMatrixd(RollPitchYawd(M_PI / 6., M_PI / 3., M_PI / 5.)),
      Vector3d(-0.2, 2, 1.5));
  mesh_M_.TransformVertices(X_MF);
  // Now we use alias mesh_F because `mesh_M_` has been transformed; its stored
  // vertices are now measured and expressed in frame F.
  const TriangleSurfaceMesh<double>& mesh_F = mesh_M_;

  const Obb obb_F = ObbMaker(mesh_F, test_vertices_).Compute();
  EXPECT_TRUE(Contain(obb_F, mesh_F, test_vertices_));
  // The threshold was set empirically. It is much smaller (about 9.7 m³)
  // than the previous test before vertex transform (about 27 m³).
  // We use about 10-percent tolerance (0.9 m³) because the numerical
  // optimization varies among different computing platforms.
  EXPECT_NEAR(obb_F.CalcVolume(), 9.741, 0.9);

  // Empirical tolerance for comparing unit vectors from characteristic
  // equation AX = λX and rigid transform.
  double kEps = 100. * std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(CompareMatrices(obb_F.pose().GetAsMatrix4(),
                               (X_MF * obb_M.pose()).GetAsMatrix4(), kEps));
}

// TODO(DamrongGuoy): Update or remove this test if we improve the algorithm.
//  Currently our oriented box can be worse than an axis-aligned box.

// Show that our OBB algorithm can do significantly worse than AABB.
// PCA created an initial orientation that made the OBB volume larger than
// the original AABB, and our simple optimization couldn't improve it.
//
// We use vertices of a truncated rectangular box. From the 8 vertices of
// a 6x4x2 box, we omit two vertices of a diagonal and use only 6 vertices for
// testing. We have checked that the truncated box has three distinct eigen
// values of its covariance matrix (about 0.600, 3.633, and 9.762), so PCA
// solution is unique in this case.
GTEST_TEST(ObbMakerTest, TestTruncatedBox) {
  // Resolution hint 10 is larger than the largest box dimension 6, so we
  // should get the coarsest mesh: 8 vertices, 12 triangles.
  auto surface_mesh = internal::MakeBoxSurfaceMesh<double>(Box(6, 4, 2), 10);
  ASSERT_EQ(surface_mesh.num_vertices(), 8);
  ASSERT_EQ(surface_mesh.num_triangles(), 12);
  std::set<int> test_vertices;
  for (int i = 0; i < 8; ++i) {
    const Vector3d& p_MV = surface_mesh.vertex(i);
    // Omit vertices on a diagonal.
    if (CompareMatrices(p_MV, Vector3d(-3, -2, -1), 1e-5)) continue;
    if (CompareMatrices(p_MV, Vector3d(3, 2, 1), 1e-5)) continue;
    test_vertices.insert(i);
  }
  ASSERT_EQ(test_vertices.size(), 6);

  const Obb obb = ObbMaker(surface_mesh, test_vertices).Compute();
  EXPECT_TRUE(Contain(obb, surface_mesh, test_vertices));

  // The final basis is the same as the PCA basis -- optimization couldn't
  // improve it which implies we're at a local optimum.
  const RotationMatrixd R_MB =
      ObbMakerTester<TriangleSurfaceMesh<double>>(surface_mesh, test_vertices)
          .CalcOrientationByPca();
  EXPECT_TRUE(CompareMatrices(R_MB.matrix(), obb.pose().rotation().matrix()));

  // The original box of size 6x4x2 has volume 48.0. Our oriented bounding
  // box is far from optimal. Its volume is larger than the original box.
  EXPECT_NEAR(obb.CalcVolume(), 82.281, 0.001);
  // The original box has the half-width vector (3,2,1). Our OBB is almost 20%
  // and 50% longer in width and depth.
  EXPECT_TRUE(
      CompareMatrices(obb.half_width(), Vector3d(3.552, 2.960, 0.977), 0.001));
}

// Smoke test that it works with VolumeMesh<double>.
GTEST_TEST(ObbMakerTest, TestVolumeMesh) {
  // Use a very coarse mesh.
  const VolumeMesh<double> volume_mesh =
      internal::MakeEllipsoidVolumeMesh<double>(
          Ellipsoid(1., 2., 3.), 6,
          internal::TessellationStrategy::kSingleInteriorVertex);
  std::set<int> test_vertices;
  for (int i = 0; i < volume_mesh.num_vertices(); ++i) {
    test_vertices.insert(i);
  }
  Obb obb = ObbMaker(volume_mesh, test_vertices).Compute();
  EXPECT_TRUE(Contain(obb, volume_mesh, test_vertices));
  // The threshold was set empirically. It is slightly smaller than the
  // trivial upper bound 48.0, which is the volume of the box of size 2x4x6
  // that fits the ellipsoid. We put a check that the future code will not
  // create a bigger bounding box.
  EXPECT_LT(obb.CalcVolume(), 41.317);
}

// Smoke test that it works with PolygonSurfaceMesh.
GTEST_TEST(ObbMakerTest, TestPolygonSurfaceMesh) {
  const std::string file =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  const Mesh mesh(file);
  const PolygonSurfaceMesh<double> polygon_mesh =
      internal::MakeConvexHull(mesh);
  std::set<int> test_vertices;
  for (int i = 0; i < polygon_mesh.num_vertices(); ++i) {
    test_vertices.insert(i);
  }
  Obb obb = ObbMaker(polygon_mesh, test_vertices).Compute();
  EXPECT_TRUE(Contain(obb, polygon_mesh, test_vertices));
}

// Tests API of ObbMaker that it respects the input vertex indices.
GTEST_TEST(ObbMakerTestAPI, ObbMakerCompute) {
  const TriangleSurfaceMesh<double> mesh(
      // The triangles are not relevant to the test.
      {{0, 1, 2}, {0, 3, 1}}, {Vector3d::Zero(), Vector3d::UnitX(),
                               2. * Vector3d::UnitY(), 3. * Vector3d::UnitZ()});

  const std::set<int> test_vertices{0, 1};

  const Obb obb = ObbMaker(mesh, test_vertices).Compute();

  // The mesh has four vertices, but we use only two vertices for testing.
  // Therefore, we expect the OBB to contain only the two testing vertices
  // but not the remaining vertices of the mesh.
  EXPECT_TRUE(Contain(obb, mesh, test_vertices));
  EXPECT_NEAR(obb.half_width().x(), 0.5, ObbTester::kTolerance);
  EXPECT_NEAR(obb.half_width().y(), 0., ObbTester::kTolerance);
  EXPECT_NEAR(obb.half_width().z(), 0., ObbTester::kTolerance);
  const std::set<int> remaining_vertices{2, 3};
  EXPECT_FALSE(Contain(obb, mesh, remaining_vertices));
}

// Tests calculating the bounding box volume. Due to boundary padding, the
// volume is increased from 8abc to 8((a + ε)*(b + ε)*(c+ε)), i.e.:
// 8[abc + (ab + bc + ac)ε + (a + b + c)ε² + ε³].
TEST_F(ObbTester, TestVolume) {
  Obb obb(RigidTransformd(Vector3d(-1, 2, 1)), Vector3d(2, 0.5, 2.7));
  // In this case the dominating error term is 8(ab + bc + ac)ε, which caps
  // out under kTolerance * 70.
  const double volume = obb.CalcVolume();
  EXPECT_NEAR(volume, 21.6, kTolerance * 70);
  EXPECT_GT(volume, 21.6);
  Obb zero_obb(RigidTransformd(RotationMatrixd(RollPitchYawd(
                                   M_PI / 6., M_PI / 3., M_PI / 5.)),
                               Vector3d(3, -4, 1.3)),
               Vector3d(0, 0, 0));
  // Since a, b and c are 0, only the ε³ term is left and kTolerance³ is
  // within kTolerance.
  const double zero_volume = zero_obb.CalcVolume();
  EXPECT_NEAR(zero_volume, 0, kTolerance);
  EXPECT_GT(zero_volume, 0);
}

// Tests padding the boundary of the bounding box volume.
TEST_F(ObbTester, TestPadBoundary) {
  Obb obb(RigidTransformd(Vector3d(-1, 0.5, 1)), Vector3d(1.2, 2.5, 0.3));
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
  obb = Obb(RigidTransformd(
                RotationMatrixd(RollPitchYawd(M_PI / 6., M_PI / 3., M_PI / 5.)),
                Vector3d(-300, 50, 100)),
            Vector3d(1, 2, 0.5));
  padded = Vector3d(1, 2, 0.5).array() + padding;
  // Expect the two Vector3d to be exactly equal down to the last bit.
  EXPECT_TRUE(CompareMatrices(obb.half_width(), padded));
}

// Tests the Obb-Aabb intersection.  We rely on TestObbOverlap to cover all the
// subtleties of the test. This just confirms that the Aabb is accounted for
// and reports contact. So, we'll pick a couple of arbitrary poses to trigger
// true and false results.
//
// This implicitly confirms that HasOverlap(Obb, Obb) also works. The details
// of that test have already been tested in boxes_overlap_test.cc.
GTEST_TEST(ObbTest, AabbOverlap) {
  /* Methodology:
    - Define two boxes that are definitely overlapping.
      - in a common frame, move them in opposite directions along the x-axis
        so they are just in contact.
      - Introduce two arbitrary *hierarchy* frames and define Aabb/Obb such
        that in the *world* frame, they are in the expected position (and
        orientation). */
  /* The half measures of the two boxes. */
  const Vector3d half_sizeA{0.5, 0.75, 0.125};
  const Vector3d half_sizeB{0.3, 0.6, 0.45};

  /* Produce a pair of poses such that the boxes overlap near the origin. */
  auto world_poses = [&half_sizeA, &half_sizeB](double distance) {
    const double half_distance = distance / 2;
    RigidTransformd X_WA{Vector3d{-half_sizeA.x() - half_distance, 0, 0}};
    RigidTransformd X_WB{Vector3d{half_sizeB.x() + half_distance, 0, 0}};
    return std::make_pair(X_WA, X_WB);
  };

  for (double distance : {-0.001, 0.001}) {
    const auto [X_WA, X_WB] = world_poses(distance);
    const bool expect_overlap = distance < 0;

    SCOPED_TRACE(fmt::format("distance = {}", distance));

    /* Quick reality check -- the two boxes, in their common frame overlap. */
    EXPECT_EQ(Obb::HasOverlap(Obb(X_WA, half_sizeA), Obb(X_WB, half_sizeB),
                              RigidTransformd::Identity()),
              expect_overlap);

    /* We have the target poses of the boxes in the world frame. We'll define
     hierarchy frames G and H arbitrarily but pose them in the world to
     guarantee that the final box poses are assured. */
    const RigidTransformd X_GA{
        AngleAxisd{M_PI / 7, Vector3d{1, -2, 3}.normalized()},
        Vector3d{-0.5, 3, 1.5}};
    const RigidTransformd X_WG = X_WA * X_GA.inverse();

    // B is an aabb, so R_HB = I.
    const RigidTransformd X_HB{Vector3d{-1, 1, -1}};
    const RigidTransformd X_WH = X_WB * X_HB.inverse();

    const Obb obb_G{X_GA, half_sizeA};
    const Aabb aabb_H{X_HB.translation(), half_sizeB};
    const RigidTransformd X_GH = X_WG.inverse() * X_WH;

    EXPECT_EQ(Obb::HasOverlap(obb_G, aabb_H, X_GH), expect_overlap);
  }
}

// Tests Obb-plane intersection. The vast majority of the work is done by
// the Plane class (tested elsewhere). We just need to confirm that the OBB
// describes itself in the Plane's frame correctly. To that end, we make sure
// that the obb has non-trivial values for position and orientation and that the
// transform between the hierarchy frame and plane frame is likewise
// "interesting" (we're making sure that all the bits get used properly).
GTEST_TEST(ObbTest, PlaneOverlap) {
  // We'll define the problem *in* the plane's frame, but arbitrarily pose the
  // problem in a separate frame prior to evaluation.
  const Vector3d half_width(0.25, 2.0, 1.5);

  // Box pose in the hierarchy frame H.
  const RigidTransformd X_HB(RotationMatrixd::MakeFromOneVector(
                                 Vector3d(1, 2, 3), /* axis_index= */ 2),
                             Vector3d(-1, 2, 0.5));

  // Hierarchy pose in the plane's frame. We're free to pick R_PH (such that
  // R_PB ≠ I). We have to pick p_PH such that the box's corner just touches the
  // plane.
  const auto R_PH = RotationMatrixd::MakeFromOneVector(Vector3d(-1, 2, -2), 2);

  // The "clearance" is the height the box center needs to be off the plane so
  // that one corner is exactly touching (given its relative orientation).
  const RotationMatrixd R_PB = R_PH * X_HB.rotation();
  const double clearance = R_PB.row(2).cwiseAbs().dot(half_width.transpose());
  // The plane's normal is Pz, so only the z-value in the box origin matters.
  const Vector3d p_PB(1.5, -0.5, clearance);
  const Vector3d p_HB_P = R_PH * X_HB.translation();
  const Vector3d p_PH = p_PB - p_HB_P;
  const RigidTransformd X_PH(R_PH, p_PH);

  // Pose the whole problem in the world frame with some arbitrary, non-identity
  // pose.
  const RigidTransformd X_WP(
      RotationMatrixd::MakeFromOneVector(Vector3d(-1, 2, -2), 2),
      Vector3d{3, -1, 2});
  const Vector3d Pz_W = X_WP.rotation().col(2);
  const Plane<double> plane_W(Pz_W, X_WP.translation());

  const Obb obb_H(X_HB, half_width);
  // Initialize X_WH such that the box corner exactly touches the plane. Then
  // we'll perturb it in the normal direction up by epsilon (out of contact) and
  // down again by two epsilon (into contact).
  RigidTransformd X_WH = X_WP * X_PH;

  const double kUp = 1e-6;
  // Shift frame H up.
  X_WH = RigidTransformd(Pz_W * kUp) * X_WH;
  EXPECT_FALSE(Obb::HasOverlap(obb_H, plane_W, X_WH));

  const double kDown = -2 * kUp;
  // Shift frame H down.
  X_WH = RigidTransformd(Pz_W * kDown) * X_WH;
  EXPECT_TRUE(Obb::HasOverlap(obb_H, plane_W, X_WH));
}

// Tests Obb-halfspace itersection.
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
      AngleAxisd{12 * M_PI / 7, Vector3d{1, -2, 3}.normalized()}};
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
}  // namespace geometry
}  // namespace drake
