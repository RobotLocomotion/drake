#include "drake/geometry/proximity/aabb.h"

#include <limits>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::set;
using std::vector;

// Local implementation of BvhUpdater to exercise Aabb::set_bounds().
template <typename T>
class BvhUpdater {
 public:
  static void SetBounds(Aabb* aabb, const Vector3d& lower,
                        const Vector3d& upper) {
    aabb->set_bounds(lower, upper);
  }
};

namespace {

/* This tests the constructor and that the expected values get to the expected
 fields. */
GTEST_TEST(AabbTest, Construction) {
  const Vector3d p_HB{1, 2, 3};
  const Vector3d half_size{0.75, 0.875, 1.25};
  const double expected_volume =
      2 * half_size.x() * 2 * half_size.y() * 2 * half_size.z();

  const Aabb aabb(p_HB, half_size);

  EXPECT_TRUE(CompareMatrices(aabb.center(), p_HB));
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), half_size));
  EXPECT_EQ(aabb.CalcVolume(), expected_volume);
  EXPECT_TRUE(CompareMatrices(aabb.lower(), p_HB - half_size));
  EXPECT_TRUE(CompareMatrices(aabb.upper(), p_HB + half_size));

  EXPECT_TRUE(aabb.pose().rotation().IsExactlyIdentity());
  EXPECT_TRUE(CompareMatrices(aabb.pose().translation(), p_HB));
}

/* Confirms that the private set_bounds() appropriately changes the box. */
GTEST_TEST(AabbTest, UpdateBounds) {
  // Create a baseline Aabb.
  const Vector3d p_HB{1, 2, 3};
  const Vector3d half_size{0.75, 0.875, 1.25};
  const double expected_volume =
      2 * half_size.x() * 2 * half_size.y() * 2 * half_size.z();

  Aabb aabb(p_HB, half_size);
  EXPECT_TRUE(CompareMatrices(aabb.center(), p_HB));
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), half_size));
  EXPECT_EQ(aabb.CalcVolume(), expected_volume);

  const Vector3d p_HB2 = p_HB + Vector3d{-1, 0.25, 1};
  const Vector3d half_size2 = 2 * half_size;
  BvhUpdater<double>::SetBounds(&aabb, p_HB2 - half_size2, p_HB2 + half_size2);
  EXPECT_TRUE(CompareMatrices(aabb.center(), p_HB2));
  EXPECT_TRUE(CompareMatrices(aabb.half_width(), half_size2));
  EXPECT_EQ(aabb.CalcVolume(), expected_volume * 8);
}

/* This tests the overlap query for aabb-aabb and aabb-obb. We rely on the
 tests in obb_test.cc to generally cover the underlying Gottschalk algorithm.
 It's enough test to cover colliding/non-colliding cases. */
GTEST_TEST(AabbTest, BoxOverlap) {
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

  /* Produce a pair of poses such that the boxes near the origin overlap for
   distance < 0. */
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
    EXPECT_EQ(Aabb::HasOverlap(Aabb(X_WA.translation(), half_sizeA),
                               Aabb(X_WB.translation(), half_sizeB),
                               RigidTransformd::Identity()),
              expect_overlap);

    {
      /* Two Aabb, but with more interesting frames. We'll define Frames G and H
       arbitrarily but satisfy the constraint that the final box poses are X_WA
       and X_WB, i.e., we require R_GA = R_HB = I for Aabbs. */
      const RigidTransformd X_GA{Vector3d{-0.5, 3, 1.5}};
      const RigidTransformd X_WG = X_WA * X_GA.inverse();
      const RigidTransformd X_HB{Vector3d{-1, 1, -1}};
      const RigidTransformd X_WH = X_WB * X_HB.inverse();

      const Aabb aabbA_G(X_GA.translation(), half_sizeA);
      const Aabb aabbB_H(X_HB.translation(), half_sizeB);
      const RigidTransformd X_GH = X_WG.inverse() * X_WH;

      EXPECT_EQ(Aabb::HasOverlap(aabbA_G, aabbB_H, X_GH), expect_overlap);
    }

    {
      /* Box B is now an Obb so R_HB = I is no longer a requirement. */
      const RigidTransformd X_GA{Vector3d{-0.5, 3, 1.5}};
      const RigidTransformd X_WG = X_WA * X_GA.inverse();
      const RigidTransformd X_HB{
          AngleAxisd{M_PI / 7, Vector3d{1, -2, 3}.normalized()},
          Vector3d{-1, 1, -1}};
      const RigidTransformd X_WH = X_WB * X_HB.inverse();

      const Aabb aabbA_G(X_GA.translation(), half_sizeA);
      const Obb obbB_H(X_HB, half_sizeB);
      const RigidTransformd X_GH = X_WG.inverse() * X_WH;

      EXPECT_EQ(Aabb::HasOverlap(aabbA_G, obbB_H, X_GH), expect_overlap);
    }
  }
}

GTEST_TEST(AabbTest, TestEqual) {
  const Vector3d p_HB{0.5, 0.25, -0.75};
  const Vector3d half_size{1, 2, 3};
  const Aabb a{p_HB, half_size};
  /* Equal to itself.  */
  EXPECT_TRUE(a.Equal(a));
  /* Different pose.  */
  const Aabb b{p_HB * 1.1, half_size};
  EXPECT_FALSE(a.Equal(b));
  /* Different half_width.  */
  const Aabb c{p_HB, half_size * 1.1};
  EXPECT_FALSE(a.Equal(c));
  /* Same values, different instances.  */
  const Aabb d{p_HB, half_size};
  EXPECT_TRUE(a.Equal(d));
  /* It is *exactly* equals; epsilon is insufficient.  */
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  const Aabb e(p_HB + Vector3d(kEps, 0, 0), half_size);
  EXPECT_FALSE(a.Equal(e));
}

/* Confirm that the AabbMaker successfully makes the expected bounding boxes.
 We'll do this by creating a fake mesh with carefully curated vertices (the
 elements of the mesh will be garbage). Then we'll successively build Aabb
 instances from subsets of the vertices. */
GTEST_TEST(AabbMakerTest, Compute) {
  /* The vertices are all located at box corners.

        V₃--------------V₇
       / |             / |
      /  |            /  |      z   y
     /   |           /   |      | /
   V₁--------------V₅    |      |/
   |     |         |     |      o-------x
   |    V₂---------|----V₆
   |   /           |   /
   |  /            |  /
   | /             | /
   V₀--------------V₄
   */
  const Vector3d half_size{0.5, 1.5, 1.25};
  vector<Vector3d> vertices;
  for (double x : {-1, 1}) {
    for (double y : {-1, 1}) {
      for (double z : {-1, 1}) {
        vertices.emplace_back(Vector3d{x, y, z}.cwiseProduct(half_size));
      }
    }
  }
  vector<SurfaceTriangle> faces{{0, 1, 2}};
  TriangleSurfaceMesh<double> mesh{std::move(faces), std::move(vertices)};

  ASSERT_EQ(mesh.num_vertices(), 8);

  {
    /* Case: single vertex. */
    const set<int> fit_vertices = {0};
    const Aabb::Maker<TriangleSurfaceMesh<double>> maker(mesh, fit_vertices);
    const Aabb aabb = maker.Compute();
    EXPECT_TRUE(CompareMatrices(aabb.center(), mesh.vertex(0)));
    EXPECT_TRUE(CompareMatrices(aabb.half_width(), Vector3d::Zero()));
  }

  {
    /* Case: Two vertices forming an axis-aligned edge. */
    const set<int> fit_vertices = {3, 7};
    const Aabb::Maker<TriangleSurfaceMesh<double>> maker(mesh, fit_vertices);
    const Aabb aabb = maker.Compute();
    EXPECT_TRUE(CompareMatrices(aabb.center(),
                                Vector3d{0, 1, 1}.cwiseProduct(half_size)));
    EXPECT_TRUE(CompareMatrices(aabb.half_width(),
                                Vector3d{1, 0, 0}.cwiseProduct(half_size)));
  }

  {
    /* Case: Two vertices lying completely on a plane. */
    const set<int> fit_vertices = {4, 7};
    const Aabb::Maker<TriangleSurfaceMesh<double>> maker(mesh, fit_vertices);
    const Aabb aabb = maker.Compute();
    EXPECT_TRUE(CompareMatrices(aabb.center(),
                                Vector3d{1, 0, 0}.cwiseProduct(half_size)));
    EXPECT_TRUE(CompareMatrices(aabb.half_width(),
                                Vector3d{0, 1, 1}.cwiseProduct(half_size)));
  }

  {
    /* Case: Two vertices lying diagonally across the box. */
    const set<int> fit_vertices = {3, 4};
    const Aabb::Maker<TriangleSurfaceMesh<double>> maker(mesh, fit_vertices);
    const Aabb aabb = maker.Compute();
    EXPECT_TRUE(CompareMatrices(aabb.center(), Vector3d::Zero()));
    EXPECT_TRUE(CompareMatrices(aabb.half_width(), half_size));
  }

  {
    /* Case: Once I have the minimum support for the full box (e.g., vertices
     3 and 4), adding other vertices will not change the outcome. */
    const set<int> base_set = {3, 4};
    for (int v : {0, 1, 2, 5, 6, 7}) {
      set<int> fit_vertices(base_set);
      fit_vertices.emplace(v);
      const Aabb::Maker<TriangleSurfaceMesh<double>> maker(mesh, fit_vertices);
      const Aabb aabb = maker.Compute();
      EXPECT_TRUE(CompareMatrices(aabb.center(), Vector3d::Zero()));
      EXPECT_TRUE(CompareMatrices(aabb.half_width(), half_size));
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
