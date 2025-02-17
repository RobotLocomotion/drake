#include "drake/geometry/proximity/make_box_mesh.h"

#include <algorithm>
#include <array>
#include <unordered_set>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using std::unordered_set;

namespace {
/* Subdivides a virtual cube to have up to six tetrahedra such that they
 share the diagonal v0v6 (see ordering below). We allow repeated vertices in
 the virtual cube and will skip tetrahedra with repeated vertices.

 We assume the input vertex indices are in this ordering:
   1. Four vertices v0,v1,v2,v3 of the "bottom" face in counterclockwise
      order when look from "above" the cube.
   2. Four vertices v4,v5,v6,v7 of the "top" face matching v0,v1,v2,v3,
      respectively.

                       v4-----------v7
                      /|           /|
                     / |          / |
                    /  |         /  |
                   /   |        /   |
                  /    v0------/----v3
                 /    /       /    /
                v5-----------v6   /
                |   /        |   /
                |  /         |  /
                | /          | /
                |/           |/
                v1-----------v2

 Examples of cases with repeated vertices.
 - If v0 == v3 and v1 == v2, we will have a prism instead of a cube and will
   get 3 tetrahedra instead of 6 tetrahedra.
 - If v0 == v1 == v2 == v3, we will have a pyramid instead of a cube and will
   get 2 tetrahedra instead of 6 tetrahedra.

 @note   This function is purely topological because it takes only
 indices into account without attempting to access coordinates of the vertices.
 */
std::vector<VolumeElement> SplitToTetrahedra(int v0, int v1, int v2, int v3,
                                             int v4, int v5, int v6, int v7) {
  std::vector<VolumeElement> elements;
  elements.reserve(6);
  int previous = v1;
  for (int next : {v2, v3, v7, v4, v5, v1}) {
    // Allow distinct vertex indices only.
    if (unordered_set<int>({previous, next, v0, v6}).size() == 4) {
      elements.emplace_back(previous, next, v0, v6);
    }
    previous = next;
  }
  return elements;
}

// Makes a mesh for a cube of lengths `length`. This tesselation:
//  - splits all faces into four triangles, and
//  - respects the medial axis.
template <typename T>
VolumeMesh<T> MakeBoxWithPointMa(double length) {
  // clang-format off
  std::vector<VolumeElement> elements = {
    // Every tetrahedron shares vertex 8, the MA point. Each tetrahedron has one
    // vertex from the six face-center vertices (see description of vertex 9-14
    // later) as the last vertex in each of the 4-vertex lists below. The
    // remaining two vertices in each tetrahedron form an edge (two adjacent
    // corners) of the cube (see description of vertex 0-7 later).
    //
    // Each of the six face-center vertices is used in four tetrahedra, and each
    // of the twelve edges of the cube is used in two tetrahedra: 6x4 = 12x2 =
    // 24 tetrahedra in total.
    //
    // We have verified the above properties in Paraview.
    {8, 0, 4, 11},
    {3, 8, 1,  9},
    {0, 8, 4, 13},
    {8, 7, 3, 14},
    {8, 5, 4, 10},
    {8, 0, 1,  9},
    {5, 8, 1, 14},
    {0, 8, 2,  9},
    {2, 8, 3,  9},
    {8, 4, 6, 10},
    {7, 8, 6, 10},
    {8, 5, 1, 11},
    {4, 8, 6, 13},
    {8, 5, 7, 14},
    {8, 7, 6, 12},
    {8, 6, 2, 12},
    {5, 8, 7, 10},
    {5, 8, 4, 11},
    {8, 3, 1, 14},
    {8, 2, 3, 12},
    {7, 8, 3, 12},
    {8, 0, 2, 13},
    {6, 8, 2, 13},
    {0, 8, 1, 11}};
  // clang-format on

  // clang-format off
  const T L = length / 2.0;
  std::vector<Vector3<T>> vertices  = {
    // Vertex 0 to 7 are the eight corners (±L, ±L, ±L) of the box.
    {-L, -L, -L},
    {-L, -L,  L},
    {-L,  L, -L},
    {-L,  L,  L},
    { L, -L, -L},
    { L, -L,  L},
    { L,  L, -L},
    { L,  L,  L},
    // Vertex 8 is the point MA at the origin of the box's frame.
    {0., 0., 0.},
    // Vertex 9 to 14 are the six face centers.
    {-L, 0., 0.},
    { L, 0., 0.},
    {0., -L, 0.},
    {0.,  L, 0.},
    {0., 0., -L},
    {0., 0.,  L}};
  // clang-format on

  return {std::move(elements), std::move(vertices)};
}

// Makes a mesh for a box having two lengths to be equal and smaller than the
// third length. This tesselation:
//  - splits all faces into four triangles, and
//  - respects the medial axis.
// @pre The box has a line MA.
template <typename T>
VolumeMesh<T> MakeBoxWithLineMa(const Box& box) {
  // Below, `elements` and `vertices` define a tessellation of the box in a
  // canonical frame in which the longest size is along the z-axis.

  // clang-format off
  // The medial surface partitions the box into two square pyramids on the
  // square faces of the box and four rectangular hip-roofs
  // (https://en.wikipedia.org/wiki/Hip_roof) on the four rectangular faces of
  // the box.
  //
  // These pictures are examples of such a square pyramid and a rectangular
  // hip-roof.
  //
  //           +-----------+          These pictures are intended to be
  //           | ↘       ↙ |          three dimensional with the axis of
  //           |   ↘   ↙   |          the third dimension perpendicular
  //           |     x     |          to the screen. The arrows go from
  //           |   ↗   ↖   |          lower altitude to higher altitude.
  //           | ↗       ↖ |          All arrows have the same slope 1.
  //           +-----------+
  //
  //   +--------------------------+
  //   | ↘                      ↙ |
  //   |   ↘                  ↙   |
  //   |     x--------------x     |
  //   |   ↗                  ↖   |
  //   | ↗                      ↖ |
  //   +--------------------------+
  //
  // We further subdivide the above polytopes by the six face centers (vertex 11
  // to 16) of the box and the body center (vertex 10) of the box in the
  // following ways.
  //
  // For each square pyramid, we connect a square-face center (vertex 15 or 16)
  // of the box to each of the four triangles of the pyramid, so we have 4
  // tetrahedra per pyramid. (The connectivity below shows 4 entries with vertex
  // 15 and 4 entries with vertex 16.)
  //
  // For each hip-roof (two triangles and two trapezoids in the above picture),
  // we subdivide each trapezoid into three triangles (one isosceles triangle
  // and two obtuse triangles) by the body center (vertex 10) like this:
  //
  //   +---------------------------+
  //     ↘  .                 .  ↙
  //       ↘     .       .     ↙
  //         x-------o-------x
  //                V10
  //
  // Each hip-roof becomes 8 triangles (2 trapezoids x 3 triangles/trapezoid + 2
  // triangles). To create tetrahedra, we connect a face center on the
  // rectangular face of the box (vertex 11, 12, 13, or 14) to each of the eight
  // triangles.  We have 8 tetrahedra per hip-roof. (Each of vertex 11, 12, 13,
  // and 14 appears in 8 entries in the table below.)
  //
  // In total we have 40 tetrahedra:
  //    2 pyramids x 4 tetrahedra/pyramid = 8 tetrahedra
  //    4 hip roofs x 8 tetrahedra/hip roof = 32 tetrahedra
  //
  // We have verified the connectivity manually in Paraview.
  std::vector<VolumeElement> elements = {
    { 5,   10,    4,   13},
    { 3,    1,    8,   16},
    { 7,    8,    5,   16},
    { 6,    9,    4,   12},
    { 8,    1,    5,   16},
    { 2,   10,    3,   11},
    { 6,    7,   10,   12},
    {10,    5,    4,   12},
    {10,    0,    1,   11},
    { 7,    6,   10,   14},
    { 8,    7,    5,   12},
    { 3,    8,    7,   16},
    { 6,    9,    2,   15},
    { 9,    0,    2,   15},
    { 1,    8,   10,   11},
    { 1,    3,    8,   11},
    { 8,    3,   10,   11},
    { 2,    9,   10,   11},
    { 0,    9,    2,   11},
    {10,    9,    0,   11},
    { 8,    3,    7,   14},
    { 8,    7,   10,   14},
    { 9,    6,    2,   14},
    {10,    6,    9,   14},
    {10,    2,    3,   14},
    { 3,    8,   10,   14},
    { 9,    2,   10,   14},
    { 9,    4,    0,   15},
    { 9,    6,    4,   15},
    {10,    9,    4,   13},
    { 4,    9,    0,   13},
    { 8,   10,    5,   13},
    { 1,    8,    5,   13},
    { 0,   10,    1,   13},
    { 9,   10,    0,   13},
    { 8,    1,   10,   13},
    {10,    8,    5,   12},
    { 7,    8,   10,   12},
    { 9,   10,    4,   12},
    { 6,   10,    9,   12}};
  // clang-format on

  // We first create the mesh in the canonical frame and then rotate it back to
  // the original frame.
  const Vector3d half_sizes = box.size() / 2.0;
  int z_dir;
  const T L2 = half_sizes.maxCoeff(&z_dir);
  const int x_dir = (z_dir + 1) % 3;
  const T L1 = half_sizes(x_dir);
  const T Lm = L2 - L1;  // MA half size.

  // Coordinates of the box in the canonical frame, where the z-axis is aligned
  // along the longest size.
  // clang-format off
  std::vector<Vector3<T>> vertices  = {
    // Vertex 0-7 are the eight corners (±L1, ±L1, ±L2) of the box.
    {-L1, -L1, -L2},
    {-L1, -L1,  L2},
    {-L1,  L1, -L2},
    {-L1,  L1,  L2},
    { L1, -L1, -L2},
    { L1, -L1,  L2},
    { L1,  L1, -L2},
    { L1,  L1,  L2},
    // Vertices 8 and 9 are the endpoints of the median line.
    { 0,  0,  Lm},
    { 0,  0, -Lm},
    // Vertex 10 is the center of the median line, i.e. the center of the box.
    { 0,  0,  0},
    {-L1,  0,  0},
    { L1,  0,  0},
    { 0, -L1,  0},
    { 0,  L1,  0},
    { 0,  0, -L2},
    { 0,  0,  L2}};
  // clang-format on

  if (z_dir == 0) {
    // Rotate 90 degrees around the canonical y axis.
    std::transform(vertices.begin(), vertices.end(), vertices.begin(),
                   [](const Vector3<T>& p) {
                     return Vector3<T>(-p(2), p(1), p(0));
                   });
  }

  if (z_dir == 1) {
    // Rotate 90 degrees around the canonical x axis.
    std::transform(vertices.begin(), vertices.end(), vertices.begin(),
                   [](const Vector3<T>& p) {
                     return Vector3<T>(p(0), -p(2), p(1));
                   });
  }

  return {std::move(elements), std::move(vertices)};
}

// Makes a mesh for a box for which its medial axis forms a rectangular surface.
// This tesselation:
//  - splits all faces into four triangles, and
//  - respects the medial axis.
// @pre The box has a rectangular MA.
template <typename T>
VolumeMesh<T> MakeBoxWithRectangleMa(const Box& box) {
  // Below, `elements` and `vertices` define a tessellation of the box in a
  // canonical frame in which the shortest size is along the x-axis. This x-axis
  // is then perpendicular to the plane that contains the rectangular MA.

  // clang-format off
  // The medial surface partitions the box into two frustums and four hip-roofs
  // (https://en.wikipedia.org/wiki/Hip_roof). Examples of a frustum
  // (perpendicular to the x-axis) and two hip-roofs (perpendicular to the
  // y-axis and z-axis) are shown in this orthographic picture (Mᵢ are the
  // medial vertices):
  //
  //     +--------------------------+      +---------+
  //     | ↘  Mᵢ               Mⱼ ↙ |      | ↘    ↙  |
  //     |   ↘------------------↙   |      |   ↘ ↙   |
  //     |   |                  |   |      |    |    |
  //     |   ↗------------------↖   |      |   ↗ ↖   |
  //     | ↗  Mₖ               Mₗ ↖ |      | ↗     ↖ |
  //     +--------------------------+      +---------+
  //
  //     +--------------------------+   These pictures are intended to be
  //     | ↘                      ↙ |   three-dimensional with the axis of
  //     |   x------------------x   |   the third dimension perpendicular
  //     | ↗                      ↖ |   to the screen. The arrows go from
  //     +--------------------------+   lower altitude to higher altitude.
  //                                    All arrows have the same slope, 1.
  //
  // For each of the two frustums, we cut the medial rectangle MᵢMⱼMₖMₗ (vertex
  // 8-11) into two triangles and cut each of the four trapezoids into two
  // triangles. There are 2 + 2x4 = 10 such triangles. To create tetrahedra, we
  // connect one of the two face centers on the x-axis (vertex 12 and 13) to the
  // 10 triangles. Each of vertex 12 and vertex 13 is listed in 10 entries in
  // the table below.
  //
  // For each of the four hip-roofs, we cut each of its two trapezoidal faces
  // into two triangles, so we have six triangles per hip-roof.  To create
  // tetrahedra, we connect a face center on the y-axis or z-axis (vertex 14,
  // 15, 16, or 17) to the six triangles. Each of vertex 14 to 17 is listed 6
  // times in the table below.
  //
  // In total there are 44 tetrahedra:
  //     2 frustums x 10 tetrahedra/frustum = 20 tetrahedra,
  //     4 hip-roofs x 6 tetrahedra/hip-roof = 24.
  //
  // We have manually verified the connectivity in Paraview.
  std::vector<VolumeElement> elements = {
    { 4, 11,  6, 16},
    { 8,  5,  1, 14},
    { 5,  9,  8, 17},
    { 5,  4,  8, 13},
    { 5,  8,  1, 17},
    { 0, 11,  2, 12},
    { 7,  9,  6, 13},
    { 3,  7,  9, 15},
    { 3,  2,  9, 12},
    { 9,  7,  6, 15},
    { 1,  9,  3, 17},
    { 6, 11,  2, 16},
    { 2, 11,  9, 12},
    { 9,  8,  1, 12},
    { 9,  5,  8, 13},
    { 5,  9,  7, 13},
    { 8,  9, 11, 12},
    { 8,  0,  1, 12},
    {11,  4,  6, 13},
    { 9,  1,  3, 12},
    { 4,  5,  8, 14},
    { 7,  3,  9, 17},
    { 9,  5,  7, 17},
    { 8, 11, 10, 12},
    { 0,  8, 10, 12},
    {11,  0, 10, 12},
    { 8,  9,  1, 17},
    {11,  6,  2, 15},
    {11,  9,  6, 15},
    { 2,  3,  9, 15},
    {11,  2,  9, 15},
    { 4,  0, 10, 16},
    {11,  4, 10, 16},
    {11,  0,  2, 16},
    { 0, 11, 10, 16},
    { 4,  8, 10, 14},
    { 0,  4, 10, 14},
    { 0,  8,  1, 14},
    { 8,  0, 10, 14},
    { 9, 11,  6, 13},
    { 9,  8, 11, 13},
    { 8,  4, 10, 13},
    {11,  8, 10, 13},
    { 4, 11, 10, 13}};
  // clang-format on

  // We first make a mesh for the box int its canonical frame and then rotate it
  // back to the original frame of the box.
  const Vector3d half_sizes = box.size() / 2.0;

  int x_dir;  // The MA normal is in the x_dir.
  const T L1 = half_sizes.minCoeff(&x_dir);

  // Default initialize L2 and L3 for the case x_dir == 0, when no rotation is
  // needed.
  T L2 = half_sizes(1);
  T L3 = half_sizes(2);

  if (x_dir == 2) {
    // We'll rotate from the canonical frame along y.
    // Therefore y axis length stays the same and x swaps with z.
    L2 = half_sizes(1);  // stays the same
    L3 = half_sizes(0);  // swaps x and z
  }

  if (x_dir == 1) {
    // We'll rotate from the canonical frame along z.
    // Therefore z axis length stays the same and x swaps with y.
    L3 = half_sizes(2);  // stays the same.
    L2 = half_sizes(0);  // swaps x and y.
  }

  // MA half sizes.
  const T Lm2 = L2 - L1;
  const T Lm3 = L3 - L1;

  // Coordinates of the box in the canonical frame C. Lengths L1 < L2 < L3 are
  // along Cx, Cy, Cz respectively and the MA plane is in the y-z plane (zero
  // size in x).
  // clang-format off
  std::vector<Vector3<T>> vertices  = {
    // Vertex 0-7 are at the 8 corners of the box.
    {-L1, -L2, -L3},
    {-L1, -L2,  L3},
    {-L1,  L2, -L3},
    {-L1,  L2,  L3},
    { L1, -L2, -L3},
    { L1, -L2,  L3},
    { L1,  L2, -L3},
    { L1,  L2,  L3},
    // Vertex 8-11 are at the 4 corners of the rectangular face of MA.
    { 0, -Lm2,  Lm3},
    { 0,  Lm2,  Lm3},
    { 0, -Lm2, -Lm3},
    { 0,  Lm2, -Lm3},
    // Vertex 12-17 are at the 6 face centers of the box.
    {-L1,  0,  0},
    { L1,  0,  0},
    { 0, -L2,  0},
    { 0,  L2,  0},
    { 0,  0, -L3},
    { 0,  0,  L3}};
  // clang-format on

  if (x_dir == 2) {
    // Rotate 90 degrees around the canonical y axis.
    std::transform(vertices.begin(), vertices.end(), vertices.begin(),
                   [](const Vector3<T>& p) {
                     return Vector3<T>(-p(2), p(1), p(0));
                   });
  }

  if (x_dir == 1) {
    // Rotate 90 degrees around the canonical z axis.
    std::transform(vertices.begin(), vertices.end(), vertices.begin(),
                   [](const Vector3<T>& p) {
                     return Vector3<T>(-p(1), p(0), p(2));
                   });
  }

  return {std::move(elements), std::move(vertices)};
}

}  // namespace

template <typename T>
VolumeMesh<T> MakeBoxVolumeMeshWithMaAndSymmetricTriangles(const Box& box) {
  const Vector3d half_box = box.size() / 2.;
  const double min_half_box = half_box.minCoeff();

  const Vector3d half_central_Ma_before_tolerancing =
      half_box - Vector3d::Constant(min_half_box);
  const Vector3d half_central_Ma =
      (half_central_Ma_before_tolerancing.array() >
       DistanceToPointRelativeTolerance(min_half_box))
          .select(half_central_Ma_before_tolerancing, 0.);

  // MA is zero in all directions, collapsing to a single point.
  const bool ma_is_point = half_central_Ma.x() == 0 &&
                           half_central_Ma.y() == 0 && half_central_Ma.z() == 0;
  if (ma_is_point) return MakeBoxWithPointMa<T>(box.width());

  // MA is zero in two directions, collapsing into a line.
  const bool ma_is_line =
      ((half_central_Ma.x() == 0) + (half_central_Ma.y() == 0) +
       (half_central_Ma.z() == 0)) == 2;
  if (ma_is_line) return MakeBoxWithLineMa<T>(box);

  // MA is zero in one direction only, collapsing into a rectangle.
  const bool ma_is_rectangle =
      ((half_central_Ma.x() == 0) + (half_central_Ma.y() == 0) +
       (half_central_Ma.z() == 0)) == 1;
  DRAKE_DEMAND(ma_is_rectangle);
  return MakeBoxWithRectangleMa<T>(box);
}

template <typename T>
VolumeMesh<T> MakeBoxVolumeMeshWithMa(const Box& box) {
  // Begin: Notes For Developers.
  //
  // <h1> Illustration in two dimensions </h1>
  //
  // To illustrate the idea of the algorithm, first we describe an analogy in
  // two dimensions. Given a rectangle V₀V₁V₂V₃ in ℝ², we can imagine the
  // process of growing four trapezoids, each of which is from an edge of the
  // rectangle, as shown in these three pictures:
  //
  //     V₀                         V₁     V₀                         V₁
  //     +--------------------------+      +--------------------------+
  //     | ↘----------------------↙ |      | ↘                      ↙ |
  //     | |M₀                  M₁| |      |   ↘------------------↙   |
  //     | |                      | |      |   |                  |   |
  //     | |M₂                  M₃| |      |   ↗------------------↖   |
  //     | ↗----------------------↖ |      | ↗                      ↖ |
  //     +--------------------------+      +--------------------------+
  //     V₂                         V₃     V₂                         V₃
  //
  //     V₀                         V₁
  //     +--------------------------+
  //     | ↘                      ↙ |
  //     |   ↘                  ↙   |      ↙↘↖↗ border medial lines
  //     |    M₀==============M₁    |      ==== central medial lines
  //     |   ↗                  ↖   |
  //     | ↗                      ↖ |
  //     +--------------------------+
  //     V₂                         V₃
  //
  // Notice that:
  //
  // 1. For each vertex Vᵢ of the rectangle, its virtual copy (vertex Mᵢ in the
  //    pictures) moves along an MA's line from Vᵢ towards an MA's vertex. We
  //    call this line a border medial line (drawn with arrows in the picture).
  // 2. For each edge VᵢVⱼ, its virtual copy (edge MᵢMⱼ in the pictures)
  //    shrinks and moves towards the MA's line connecting two MA's vertices
  //    in the middle of the rectangle. We call this line the central medial
  //    line (drawn with "==" in the picture). In some cases (e.g., edges M₀M₁
  //    or M₂M₃ in the above picture), MᵢMⱼ ends up to coincide with the
  //    central medial line. In other cases (e.g., edges M₀M₂ or M₁M₃ in the
  //    above picture), MᵢMⱼ shrink to a single point, which is an MA's vertex
  //    on the central medial line.
  // 3. For each trapezoid VᵢVⱼMᵢMⱼ, it grows until its edge MᵢMⱼ reaches the
  //    central medial line (either by becoming the central medial line or by
  //    shrinking to a vertex of the central medial line). Notice that two of
  //    the four trapezoids degenerate to triangles. The four (possibly
  //    degenerated) trapezoids partition the rectangle V₀V₁V₂V₃.
  //
  // In two dimensions, the algorithm would partition the rectangle into four
  // blocks, each of which is a (possibly degenerated) trapezoid. Then, we
  // can subdivide each block into triangles for the output mesh.
  //
  // To handle both the normal trapezoids and the degenerated trapezoids
  // seamlessly, we can use two duplicated vertices of a trapezoid to
  // represent a triangle. In other words, the MA-partitioning algorithm
  // always gives virtual trapezoids VᵢVⱼMᵢMⱼ, and the triangle-subdivision
  // subroutine generates triangles VᵢVⱼMᵢ and VⱼMᵢMⱼ, and filters out the
  // triangle VⱼMᵢMⱼ when Mᵢ=Mⱼ.
  //
  // In summary, this is the algorithm in two dimensions:
  //
  //   For each edge VᵢVⱼ of the rectangle {
  //     Create a virtual trapezoid VᵢVⱼMᵢMⱼ, where Mᵢ and Mⱼ are the
  //     matching MA's vertices of Vᵢ and Vⱼ, respectively.
  //     Subdivide the virtual trapezoid VᵢVⱼMᵢMⱼ into one or two triangles.
  //   }
  //
  // Notice that if we are given a square instead of a rectangle, all the
  // virtual trapezoids would degenerate to triangles. Therefore, the
  // triangle-subdivision subroutine will generate one triangle for each
  // virtual trapezoid.
  //
  // This algorithm in two dimensions generate four and six triangles
  // in the output meshes for the input square and rectangle, respectively.
  //
  //
  // <h1> Algorithm in three dimensions </h1>
  //
  // This is the main idea of our implementation in three dimensions analogous
  // to the example in two dimensions above.
  //
  //   For each face VᵢVⱼVₖVₗ of the input box {
  //     Create a virtual frustum VᵢVⱼVₖVₗMᵢMⱼMₖMₗ, where Mᵢ, Mⱼ, Mₖ, Mₗ
  //     are the matching MA's vertices of Vᵢ, Vⱼ, Vₖ, Vₗ, respectively.
  //     Subdivide each virtual frustum (topological cube) into six
  //     tetrahedra and filter out tetrahedra with duplicated vertices.
  //   }
  //
  // In this discussion, a frustum is a three-dimensional polytope with two
  // parallel rectangular faces (one face is smaller than the other) and four
  // trapezoidal faces connecting the two rectangular faces. The following
  // pictures show the top views of a 6-face frustum, a frustum that
  // degenerates to a 5-face (topological) "triangular prism", and a frustum
  // that degenerates to a 5-face right square pyramid.
  //
  //     Vᵢ                         Vⱼ     Vᵢ                         Vⱼ
  //     +--------------------------+      +--------------------------+
  //     | ↘  Mᵢ               Mⱼ ↙ |      | ↘                      ↙ |
  //     |   ↘------------------↙   |      |   ↘                  ↙   |
  //     |   |                  |   |      |     x--------------x     |
  //     |   ↗------------------↖   |      |   ↗                  ↖   |
  //     | ↗  Mₖ               Mₗ ↖ |      | ↗                      ↖ |
  //     +--------------------------+      +--------------------------+
  //     Vₖ                         Vₗ     Vₖ                         Vₗ
  //
  //             Vᵢ          Vⱼ
  //             +-----------+          These pictures are intended to be
  //             | ↘       ↙ |          three dimensional with the axis of
  //             |   ↘   ↙   |          the third dimension perpendicular
  //             |     x     |          to the screen. The arrows go from
  //             |   ↗   ↖   |          lower altitude to higher altitude.
  //             | ↗       ↖ |          All arrows have the same slope 1.
  //             +-----------+
  //             Vₖ          Vₗ
  //
  // In all cases, we allow the virtual frustum to have duplicated vertices
  // in order to represent the degenerated ones (prisms and pyramids)
  // seamlessly. (A prism is a virtual frustum with Mᵢ = Mₖ and Mⱼ = Mₗ. A
  // pyramid is a virtual frustum with  Mᵢ = Mⱼ = Mₖ = Mₗ.)
  //
  // End: Notes for Developers.

  // The mesh vertices comprise the eight box vertices and up to four unique
  // MA's vertices inside the box. Later each virtual frustum will comprise
  // eight (possibly with duplication) indices into mesh_vertices.
  std::vector<Vector3<T>> mesh_vertices;
  mesh_vertices.reserve(12);

  // The eight mesh vertices of the box are indexed by the 2x2x2 array `v`,
  // where the i,j,k indices for v[i][j][k] correspond to the x,y,z
  // directions of the box's frame, respectively.
  //
  //         v001------v011
  //        /|        /|           Z
  //       / |       / |           |
  //      /  v000---/--v010        |
  //     /  /      /  /            o------Y  The box's frame's origin is
  //    /  /      /  /            /          actually at the center of the box.
  //   v101------v111            /
  //   | /       | /            X
  //   |/        |/
  //   v100------v110
  //
  int v[2][2][2];
  const Vector3d half_box = box.size() / 2.;
  for (const int i : {0, 1}) {
    const double x(i == 0 ? -half_box.x() : half_box.x());
    for (const int j : {0, 1}) {
      const double y(j == 0 ? -half_box.y() : half_box.y());
      for (const int k : {0, 1}) {
        const double z(k == 0 ? -half_box.z() : half_box.z());
        v[i][j][k] = static_cast<int>(mesh_vertices.size());
        mesh_vertices.emplace_back(x, y, z);
      }
    }
  }
  // Virtually we pretend that there are 8 MA's vertices--corresponding
  // to the 8 box vertices--even though actually there are at most 4 unique
  // MA's vertices (and could be as low as only 1) inside the box. The 8
  // virtual MA's vertices are represented by the 2x2x2 array `m`, where the
  // virtual MA's vertex m[i][j][k] corresponds to the box's vertex
  // v[i][j][k]. Some of the m[i][j][k]'s are duplicated by design.
  int m[2][2][2];
  const double min_half_box = half_box.minCoeff();
  // Similar to half_box describing the half dimensions of the box,
  // half_central_Ma describes the half dimensions of the central piece of MA.
  // Unlike half_box having all three non-zero components, half_central_Ma
  // always has at least one zero-valued component by design. It has one,
  // two, or three zero-valued components when the central piece of MA is a
  // rectangle, a line segment, or a point, respectively. We will use the
  // zero-valued components of half_central_Ma as indicators of duplicated
  // m[i][j][k].
  const Vector3d half_central_Ma_before_tolerancing =
      half_box - Vector3d::Constant(min_half_box);
  const Vector3d half_central_Ma =
      (half_central_Ma_before_tolerancing.array() >
       DistanceToPointRelativeTolerance(min_half_box))
          .select(half_central_Ma_before_tolerancing, 0.);
  for (const int i : {0, 1}) {
    const double x = i == 0 ? -half_central_Ma.x() : half_central_Ma.x();
    for (const int j : {0, 1}) {
      const double y = j == 0 ? -half_central_Ma.y() : half_central_Ma.y();
      for (const int k : {0, 1}) {
        const double z = k == 0 ? -half_central_Ma.z() : half_central_Ma.z();
        // Zero-valued components of half_central_Ma indicate duplication of
        // the virtual MA's vertices m[i][j][k].
        const bool duplicate_in_i = i == 1 && half_central_Ma.x() == 0;
        const bool duplicate_in_j = j == 1 && half_central_Ma.y() == 0;
        const bool duplicate_in_k = k == 1 && half_central_Ma.z() == 0;
        // Since the loops are executed in lexicographic order of (i,j,k),
        // we know that the assignment of a potential duplicated entry
        // m[1][j][k], m[i][1][k], or m[i][j][1] must be preceded already by
        // the assignment of m[0][j][k], m[i][0][k], or m[i][j][0],
        // respectively. There is no uninitialized-memory read of m[][][].
        m[i][j][k] = duplicate_in_i   ? m[0][j][k]
                     : duplicate_in_j ? m[i][0][k]
                     : duplicate_in_k ? m[i][j][0]
                                      : static_cast<int>(mesh_vertices.size());
        if (!duplicate_in_i && !duplicate_in_j && !duplicate_in_k) {
          mesh_vertices.emplace_back(x, y, z);
        }
      }
    }
  }
  // 8 box vertices + at most 4 MA's vertices.
  DRAKE_DEMAND(mesh_vertices.size() <= 12);

  std::vector<VolumeElement> mesh_elements;
  mesh_elements.reserve(24);
  auto append =
      [&mesh_elements](const std::vector<VolumeElement>& new_elements) {
        mesh_elements.insert(mesh_elements.end(), new_elements.begin(),
                             new_elements.end());
      };

  // We design the ordering of the 8 vertices of each virtual frustum, so
  // that all SplitToTetrahedra() of all virtual frusta are conforming.
  // Here conforming means a tetrahedron of one virtual frustum and another
  // tetrahedron of another virtual frustum intersect in their shared face,
  // or shared edge, or shared vertex, i.e., there is no partial overlap.
  //
  // This picture is useful for visualizing construction of the six virtual
  // frustra. The virtual vertices m[i][j][k] are supposed to be inside the
  // box, but we draw them outside the box for clarity. Here we draw the
  // m[i][j][k] as 8 separated vertices, even though some of them are
  // actually duplicated.
  //
  //         v001------v011
  //        /|        /|                m001--m011           Z
  //       / |       / |               /|    /|              |
  //      /  v000---/--v010           / m000/-m010           |
  //     /  /      /  /              / /   / /               o------Y
  //    /  /      /  /              m101--m111              /
  //   v101------v111               |/    |/               /
  //   | /       | /                m100--m110            X
  //   |/        |/
  //   v100------v110
  //

  // front face (+X)
  append(SplitToTetrahedra(m[1][0][0], m[1][1][0], m[1][1][1], m[1][0][1],
                           v[1][0][0], v[1][1][0], v[1][1][1], v[1][0][1]));
  // back face (-X)
  append(SplitToTetrahedra(m[0][0][0], m[0][0][1], m[0][1][1], m[0][1][0],
                           v[0][0][0], v[0][0][1], v[0][1][1], v[0][1][0]));
  // right face (+Y)
  append(SplitToTetrahedra(m[0][1][0], m[0][1][1], m[1][1][1], m[1][1][0],
                           v[0][1][0], v[0][1][1], v[1][1][1], v[1][1][0]));
  // left face (-Y)
  append(SplitToTetrahedra(m[0][0][0], m[1][0][0], m[1][0][1], m[0][0][1],
                           v[0][0][0], v[1][0][0], v[1][0][1], v[0][0][1]));
  // top face (+Z)
  append(SplitToTetrahedra(m[0][0][1], m[1][0][1], m[1][1][1], m[0][1][1],
                           v[0][0][1], v[1][0][1], v[1][1][1], v[0][1][1]));
  // bottom face (-Z)
  append(SplitToTetrahedra(m[0][0][0], m[0][1][0], m[1][1][0], m[1][0][0],
                           v[0][0][0], v[0][1][0], v[1][1][0], v[1][0][0]));
  return {std::move(mesh_elements), std::move(mesh_vertices)};
}

int CalcSequentialIndex(int i, int j, int k, const Vector3<int>& num_vertices) {
  DRAKE_DEMAND(0 <= i && i < num_vertices.x());
  DRAKE_DEMAND(0 <= j && j < num_vertices.y());
  DRAKE_DEMAND(0 <= k && k < num_vertices.z());
  return i * num_vertices.y() * num_vertices.z() + j * num_vertices.z() + k;
}

template <typename T>
std::vector<Vector3<T>> GenerateVertices(const Box& box,
                                         const Vector3<int>& num_vertices) {
  const T half_x = box.width() / T(2);
  const T half_y = box.depth() / T(2);
  const T half_z = box.height() / T(2);
  const auto x_coords =
      VectorX<T>::LinSpaced(num_vertices.x(), -half_x, half_x);
  const auto y_coords =
      VectorX<T>::LinSpaced(num_vertices.y(), -half_y, half_y);
  const auto z_coords =
      VectorX<T>::LinSpaced(num_vertices.z(), -half_z, half_z);

  std::vector<Vector3<T>> vertices;
  vertices.reserve(num_vertices.x() * num_vertices.y() * num_vertices.z());
  // The order of nested i-loop, j-loop, then k-loop makes the sequence of
  // vertices consistent with CalcSequentialIndex.
  for (int i = 0; i < num_vertices.x(); ++i) {
    for (int j = 0; j < num_vertices.y(); ++j) {
      for (int k = 0; k < num_vertices.z(); ++k) {
        vertices.emplace_back(x_coords[i], y_coords[j], z_coords[k]);
      }
    }
  }
  return vertices;
}

void AddSixTetrahedraOfCell(const Vector3<int>& lowest,
                            const Vector3<int>& num_vertices,
                            std::vector<VolumeElement>* elements) {
  const int i = lowest.x();
  const int j = lowest.y();
  const int k = lowest.z();
  // Get the sequential indices of the eight vertices of the rectangular cell.
  int v[8];
  int s = 0;
  for (int l = 0; l < 2; ++l)
    for (int m = 0; m < 2; ++m)
      for (int n = 0; n < 2; ++n)
        v[s++] = CalcSequentialIndex(i + l, j + m, k + n, num_vertices);
  // The following picture shows where vertex vₛ (for `v[s]` above) locates
  // in the rectangular cell.  The I-, J-, K-axes show the direction of
  // increasing i, j, k indices.
  //
  //               v₁     v₃
  //               ●------●
  //              /|     /|
  //             / |  v₇/ |
  //         v₅ ●------●  |
  //            |  |   |  |
  //            |  ●---|--● v₂
  //            | /v₀  | /
  //            |/     |/
  //    +K   v₄ ●------● v₆
  //     |
  //     |
  //     o------+J
  //    /
  //   /
  // +I
  //
  // The following table subdivides the rectangular cell into six tetrahedra.
  // Refer to the picture above to determine which four vertices form a
  // tetrahedron. The six tetrahedra form a cycle around the diagonal v₀v₇
  // of the cell. Refer to:
  // http://www.baumanneduard.ch/Splitting%20a%20cube%20in%20tetrahedras2.htm
  // clang-format off
  const int tetrahedron[6][4]{
      {v[0], v[7], v[4], v[6]},
      {v[0], v[7], v[6], v[2]},
      {v[0], v[7], v[2], v[3]},
      {v[0], v[7], v[3], v[1]},
      {v[0], v[7], v[1], v[5]},
      {v[0], v[7], v[5], v[4]}};
  // clang-format on
  // The above table guarantees that adjacent rectangular cells will be
  // subdivided in a consistent way, i.e., both cells will pick the same
  // diagonal of their shared rectangular face.
  for (int t = 0; t < 6; ++t) elements->emplace_back(tetrahedron[t]);
}

std::vector<VolumeElement> GenerateElements(const Vector3<int>& num_vertices) {
  std::vector<VolumeElement> elements;
  const Vector3<int> num_cell = num_vertices - Vector3<int>(1, 1, 1);
  elements.reserve(6 * num_cell.x() * num_cell.y() * num_cell.z());
  for (int i = 0; i < num_cell.x(); ++i) {
    for (int j = 0; j < num_cell.y(); ++j) {
      for (int k = 0; k < num_cell.z(); ++k) {
        AddSixTetrahedraOfCell(Vector3<int>(i, j, k), num_vertices, &elements);
      }
    }
  }
  return elements;
}

template <typename T>
VolumeMesh<T> MakeBoxVolumeMesh(const Box& box, double resolution_hint) {
  // TODO(DamrongGuoy): Generate the mesh with rigid core at medial axis or
  //  offset surface (issue #11906) and remove the "@note" above.
  DRAKE_DEMAND(resolution_hint > 0.);
  // Number of vertices in x-, y-, and z- directions.  In each direction,
  // there is one more vertices than cells.
  const Vector3<int> num_vertices{
      1 + static_cast<int>(ceil(box.width() / resolution_hint)),
      1 + static_cast<int>(ceil(box.depth() / resolution_hint)),
      1 + static_cast<int>(ceil(box.height() / resolution_hint))};

  std::vector<Vector3<T>> vertices = GenerateVertices<T>(box, num_vertices);

  std::vector<VolumeElement> elements = GenerateElements(num_vertices);

  return VolumeMesh<T>(std::move(elements), std::move(vertices));
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&MakeBoxVolumeMesh<T>, &MakeBoxVolumeMeshWithMa<T>,
     MakeBoxVolumeMeshWithMaAndSymmetricTriangles<T>));

}  // namespace internal
}  // namespace geometry
}  // namespace drake
