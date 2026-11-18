#include "drake/geometry/proximity/coal_backend.h"

#include <array>
#include <vector>

#include <gtest/gtest.h>

/* @file Direct tests of the pieces of CoalBackend that Drake's proximity
 queries cannot observe.

 MakeFaces() is the notable one. Coal's Convex stores the polygon list but,
 as Drake constructs it, derives nothing from it: the support function reads
 the vertex array, so distance and penetration results are unchanged even if
 the triangle list is wound backwards or empty. That is easy to mistake for
 "the triangulation is covered by the engine tests". It is not covered by
 them and cannot be, so it is tested here as the pure function it is.

 What the engine *can* observe is the vertex count, which Coal's constructor
 takes and FCL's infers; proximity_engine_coal_test covers that. */

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Faces = CoalBackend::FaceData;

/* Spells a triangle as a triple so that failures print readably. */
std::vector<std::array<int, 3>> AsTriples(const Faces& faces) {
  std::vector<std::array<int, 3>> out;
  for (const auto& t : faces) {
    out.push_back({static_cast<int>(t[0]), static_cast<int>(t[1]),
                   static_cast<int>(t[2])});
  }
  return out;
}

/* PolygonSurfaceMesh spells its faces [count, v0, v1, ..., count, ...]. */

GTEST_TEST(CoalMakeFacesTest, TrianglePassesThrough) {
  const std::vector<int> face_data{3, 7, 8, 9};
  EXPECT_EQ(AsTriples(CoalBackend::MakeFaces(face_data)),
            (std::vector<std::array<int, 3>>{{7, 8, 9}}));
}

/* A quad fans into two triangles around its first vertex, and each triangle
 keeps the polygon's own vertex order. Winding is unobservable through the
 queries, so this is the only place it is pinned. */
GTEST_TEST(CoalMakeFacesTest, QuadFansAroundItsFirstVertex) {
  const std::vector<int> face_data{4, 10, 11, 12, 13};
  EXPECT_EQ(AsTriples(CoalBackend::MakeFaces(face_data)),
            (std::vector<std::array<int, 3>>{{10, 11, 12}, {10, 12, 13}}));
}

GTEST_TEST(CoalMakeFacesTest, PentagonFansIntoThree) {
  const std::vector<int> face_data{5, 0, 1, 2, 3, 4};
  EXPECT_EQ(AsTriples(CoalBackend::MakeFaces(face_data)),
            (std::vector<std::array<int, 3>>{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}}));
}

/* Faces of mixed arity, concatenated, are each fanned independently; nothing
 leaks from one polygon's fan into the next. */
GTEST_TEST(CoalMakeFacesTest, MixedAritiesAreIndependent) {
  const std::vector<int> face_data{3, 0, 1, 2, 4, 3, 4, 5, 6, 3, 7, 8, 9};
  EXPECT_EQ(AsTriples(CoalBackend::MakeFaces(face_data)),
            (std::vector<std::array<int, 3>>{
                {0, 1, 2}, {3, 4, 5}, {3, 5, 6}, {7, 8, 9}}));
}

/* An n-gon yields n - 2 triangles, so a closed mesh's triangle count is the
 sum over its faces. A cube of quads -- which is what quad_cube.obj becomes
 -- is six quads and therefore twelve triangles. */
GTEST_TEST(CoalMakeFacesTest, TriangleCount) {
  std::vector<int> face_data;
  int next_vertex = 0;
  for (int i = 0; i < 6; ++i) {
    face_data.push_back(4);
    for (int k = 0; k < 4; ++k) face_data.push_back(next_vertex++);
  }
  EXPECT_EQ(CoalBackend::MakeFaces(face_data).size(), 12);
  EXPECT_EQ(CoalBackend::CountFaces(CoalBackend::MakeFaces(face_data), 6), 12);
}

/* Empty in, empty out -- not a silent single degenerate triangle. */
GTEST_TEST(CoalMakeFacesTest, EmptyInput) {
  EXPECT_TRUE(CoalBackend::MakeFaces({}).empty());
}

/* Note: MakeFaces() DRAKE_DEMANDs a well-formed encoding -- a count that
 runs off the end, or a face of fewer than three vertices. Those guard an
 invariant of the PolygonSurfaceMesh Drake itself built, not user input, so
 they abort rather than throw and are not exercised here. */

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
