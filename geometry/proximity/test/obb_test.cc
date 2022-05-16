#include "drake/geometry/proximity/obb.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

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

  // Case 1: (rank 3, multiplicty 1).
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
        mesh_M_(MakeBoxSurfaceMesh<double>(Box(2, 4, 6), 10)) {}

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
  const RotationMatrixd R_MB =
      RotationMatrixd::MakeFromOrthonormalColumns(
          Vector3d(1., -0.5, -0.5).normalized(),
          Vector3d(0., 1., -1.).normalized(),
          Vector3d(1., 1., 1.).normalized());

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
      MakeEllipsoidSurfaceMesh<double>(Ellipsoid(1., 2., 3.), 6);
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
        mesh_M_(MakeSphereSurfaceMesh<double>(Sphere(1.5), 3)),
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
  // The threshold was set empirically. It is much smaller than the
  // previous test before vertex transform.
  EXPECT_NEAR(obb_F.CalcVolume(), 9.741, 0.001);

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
  auto surface_mesh = MakeBoxSurfaceMesh<double>(Box(6, 4, 2), 10);
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
  const VolumeMesh<double> volume_mesh = MakeEllipsoidVolumeMesh<double>(
      Ellipsoid(1., 2., 3.), 6, TessellationStrategy::kSingleInteriorVertex);
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

// Tests API of ObbMaker that it respects the input vertex indices.
GTEST_TEST(ObbMakerTestAPI, ObbMakerCompute) {
  const TriangleSurfaceMesh<double> mesh(
      // The triangles are not relevant to the test.
      {{0, 1, 2}, {0, 3, 1}},
      {Vector3d::Zero(), Vector3d::UnitX(), 2. * Vector3d::UnitY(),
       3. * Vector3d::UnitZ()});

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

  // We define p_BqBo in Frame A from box b's minimum corner Q to its center.
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

// Tests whether OBBs overlap. We use *this* test to completely test the
// BoxesOverlap function. Therefore, there are 15 cases to test, each covering
// a separating axis between the two bounding boxes. The first 3 cases use the
// axes of Frame A, the next 3 cases use the axes of Frame B, and the remaining
// 9 cases use the axes defined by the cross product of axes from Frame A and
// Frame B. We also test that it is robust for the case of parallel boxes.
GTEST_TEST(ObbTest, TestObbOverlap) {
  // Frame strategy. For the canonical frame A of box `a` and the
  // canonical frame B of box `b`, we want to control the pose X_AB between
  // the two boxes. However, we need to give HasOverlap() the pose X_GH
  // between the hierarchy frames G and H to which box `a` and box `b`
  // belong. Our strategy is to control the tests through X_AB and then compose
  // X_GH as:
  //                 X_GH = X_GA * X_AB * X_BH.

  // Frame A of box `a` is arbitrarily posed in the hierarchy frame G.
  const RigidTransformd X_GA{
      RotationMatrixd(RollPitchYawd(2. * M_PI / 3., M_PI_4, -M_PI / 3.)),
      Vector3d(1, 2, 3)};
  // Frame B of box `b` is arbitrarily posed in the hierarchy frame H.
  const RigidTransformd X_HB{
      RotationMatrixd(RollPitchYawd(M_PI_4, M_PI / 5., M_PI / 6.)),
      Vector3d(2, 0.5, 4)};
  const RigidTransformd X_BH = X_HB.inverse();

  // One box is fully contained in the other and they are parallel. We make
  // them parallel by setting X_AB to identity.
  RigidTransformd X_AB = RigidTransformd::Identity();
  Obb a(X_GA, Vector3d(1, 2, 1));
  Obb b(X_HB, Vector3d(0.5, 1, 0.5));
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

// Tests the Obb-Aabb intersection.  We rely on TestObbOverlap to cover all the
// subtleties of the test. This just confirms that the Aabb is accounted for
// and reports contact. So, we'll pick a couple of arbitrary poses to trigger
// true and false results.
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
