#include <future>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/geometry/shape_specification.h"

/* This tests the race conditions for computing the convex hull on Mesh and
 Convex.

 This test is partially attempting to prevent regression in the logic to prevent
 race conditions in *initializing* the convex hull. However, it serves a second
 purpose as well. The computation of a convex hull depends on three libraries:
 qhull, VTK, and tinyobjloader. We also want to see if those libraries have
 limitations (e.g., globals) that would prevent computation of convex hulls in
 parallel.

 These tests simply dispatch a number of parallel worker threads to compute the
 convex hull. We rely on sanitizing tools to inform us if the effort revealed
 race conditions and the like.

 Because Mesh and Convex share a common mechanism for handling initialization,
 we only need to test against one shape type. */

namespace drake {
namespace geometry {
namespace {

constexpr int kNumThreads = 4;

void EvaluateConvexHullInParallel(const Mesh& mesh) {
  auto get_convex_hull = [&mesh]() {
    return &mesh.GetConvexHull();
  };

  std::vector<std::future<const PolygonSurfaceMesh<double>*>> futures;
  for (int i = 0; i < kNumThreads; ++i) {
    futures.push_back(std::async(std::launch::async, get_convex_hull));
  }

  // Do the work and collect the results.
  std::vector<const PolygonSurfaceMesh<double>*> pointers;
  for (auto& future : futures) {
    pointers.push_back(future.get());
  }

  // All pointers match.
  ASSERT_EQ(ssize(pointers), kNumThreads);
  ASSERT_NE(pointers[0], nullptr);
  for (int i = 1; i < kNumThreads; ++i) {
    ASSERT_EQ(pointers[0], pointers[i]);
  }
}

GTEST_TEST(ShapeSpecificationThreadTest, Obj) {
  // We want to use a heavyweight mesh file to extend the time to compute the
  // convex hull.
  const Mesh mesh(
      FindRunfile("drake_models/dishes/assets/plate_8in_col.obj").abspath);

  EvaluateConvexHullInParallel(mesh);
}

GTEST_TEST(ShapeSpecificationThreadTest, VolumeVtk) {
  // We want to use a heavyweight mesh file to extend the time to compute the
  // convex hull.
  const Mesh mesh(FindResourceOrThrow(
      "drake/examples/multibody/deformable/models/torus.vtk"));

  EvaluateConvexHullInParallel(mesh);
}

GTEST_TEST(ShapeSpecificationThreadTest, Gltf) {
  // We want to use a heavyweight mesh file to extend the time to compute the
  // convex hull.
  const std::string full_name =
      FindRunfile("drake_models/tri_homecart/assets/homecart_basecart.gltf")
          .abspath;
  const Mesh mesh(full_name);

  EvaluateConvexHullInParallel(mesh);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
