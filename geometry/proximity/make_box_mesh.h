#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Generates a tetrahedral volume mesh of a given box by medial axis (MA)
 subdivision. The box is subdivided into blocks using its MA, and then the
 blocks are subdivided into tetrahedra. Using this mesh with MeshFieldLinear,
 we can represent the distance-to-boundary function φ of the box accurately and
 economically. The same thing is true for a hydroelastic pressure field that
 is a constant scaling of φ.

 The box's MA is the set of all points having more than one closest point on
 the box's boundary. (See https://en.wikipedia.org/wiki/Medial_axis for more
 about the medial axis.)

 The following picture schematically shows the analogy of MA in two dimensions.
 MA of the rectangle (drawn with "--", "|", and "+") consists of the (open)
 line segments (drawn with arrows) from each of the rectangle's vertices to a
 medial vertex (drawn with "o") and the (closed) line segment (drawn with "==")
 connecting the two medial vertices.

      U                          V                    U           V
      +--------------------------+                    +-----------+
      | ↘                      ↙ |                    | ↘       ↙ |
      |   ↘                  ↙   |                    |   ↘   ↙   |
      |  M₀ o==============o M₁  |                    |  M₀ o     |
      |   ↗                  ↖   |                    |   ↗   ↖   |
      | ↗                      ↖ |                    | ↗       ↖ |
      +--------------------------+                    +-----------+
  In 2D, a rectangle's MA has 2 vertices.   In 2D, a square's MA has 1 vertex.

 In three dimensions, a box B's MA has one, two, or four vertices Mᵢ at
 the points having four or more closest points on the box's boundary.
 The MA comprises (L for set of lines, P for set of polygons):

 L1. the eight (open) line segments from B's vertex to MA's vertex,
 L2. zero, one, or four line segments (aligned to B's frame) connecting MA's
     vertices Mᵢ and Mⱼ together,
 P3. the twelve polygons that are isosceles trapezoids or isosceles triangles
     (Each polygon is bounded by an edge VᵢVⱼ of the box B, two lines VᵢMₖ and
     VⱼMₗ in L1, and, for trapezoids, the line MₖMₗ in L2, i.e., Mₖ ≠ Mₗ),
 P4. possibly one rectangle or square of four MA's vertices bounded by the
     four lines in L2 (The rectangle degenerates to one MA's vertex for a
     cube and the line in L2 for a long box with two small square faces).

 The following examples describe MAs of boxes with various shapes.

 Example 1. MA of a box with three distinct dimensions bz < bx < by comprises
   four isosceles triangles (not shown), eight isosceles trapezoids (not shown),
   and one rectangle M₀M₁M₂M₃, as shown in this picture:

          V₇----------------------------------------------V₆
         /|                                              /|
        / |                                             / |
       /  |                                            /  |      z
      /   |    M₃---------------------------------M₂  /   |      |
     /    V₃--/----------------------------------/---/----V₂     |
    /    /   /                                  /   /    /       o-------y
   V₄----------------------------------------------V₅   /       /
   |   /   /                                  /    |   /       /
   |  /   M₀---------------------------------M₁    |  /       x
   | /                                             | /
   |/                                              |/
   V₀----------------------------------------------V₁

   In this example, the MA's faces are:
     - the triangular faces (Vᵢ, Mⱼ, Vₖ):
       (V₀, M₀, V₄), (V₁, M₁, V₅), (V₂, M₂, V₆), (V₃, M₃, V₇),
     - the trapezoidal faces (Vᵢ, Mⱼ, Mₖ, Vₗ):
       (V₀, M₀, M₃, V₃), (V₄, M₀, M₃, V₇), (V₁, M₁, M₀, V₀), (V₅, M₁, M₀, V₄),
       (V₂, M₂, M₁, V₁), (V₆, M₂, M₁, V₅), (V₃, M₃, M₂, V₂), (V₇, M₃, M₂, V₆),
     - and the rectangular face: (M₀, M₁, M₂, M₃).

 Example 2. MA of a long box with two small square faces (the box dimensions
   are bz = bx < by) comprises eight isosceles triangles (not shown) and four
   isosceles trapezoids (not shown) that share the medial edge M₀M₁, as shown
   in this picture:

        V₇----------------------------------------V₆
       / |                                       / |
      /  |                                      /  |      z
     /   |                                     /   |      |
   V₄----------------------------------------V₅    |      |
   |     |   M₀------------------------M₁    |     |      o-------y
   |    V₃-----------------------------------|----V₂     /
   |   /                                     |   /      /
   |  /                                      |  /      x
   | /                                       | /
   V₀----------------------------------------V₁

   In this example, the MA's faces are:
     - the triangular faces (Vᵢ, Mⱼ, Vₖ):
       (V₀, M₀, V₄), (V₁, M₁, V₅), (V₂, M₁, V₆), (V₃, M₀, V₇),
       (V₀, M₀, V₃), (V₄, M₀, V₇), (V₂, M₁, V₁), (V₆, M₁, V₅),
     - and the trapezoidal faces (Vᵢ, Mⱼ, Mₖ, Vₗ):
       (V₁, M₁, M₀, V₀), (V₅, M₁, M₀, V₄), (V₃, M₀, M₁, V₂), (V₇, M₀, M₁, V₆).

 Example 3. MA of a cube comprises twelve isosceles triangles (not shown)
   sharing the medial vertex M₀, as shown in this picture:

        V₇--------------V₆
       / |             / |
      /  |            /  |      z
     /   |           /   |      |
   V₄--------------V₅    |      |
   |     |   M₀    |     |      o-------y
   |    V₃---------|----V₂     /
   |   /           |   /      /
   |  /            |  /      x
   | /             | /
   V₀--------------V₁

  In this example, the MA's faces are the triangular faces (Vᵢ, M₀, Vₖ):
       (V₀, M₀, V₄), (V₁, M₀, V₅), (V₂, M₀, V₆), (V₃, M₀, V₇),
       (V₀, M₀, V₃), (V₄, M₀, V₇), (V₂, M₀, V₁), (V₆, M₀, V₅),
       (V₁, M₀, V₀), (V₅, M₀, V₄), (V₃, M₀, V₂), (V₇, M₀, V₆).

 The output mesh will have these properties:

   1. The generated vertices are unique. There are no repeating vertices in
      the list of vertex coordinates.
   2. The generated tetrahedra are _conforming_. Two tetrahedra intersect in
      their shared face, or shared edge, or shared vertex, or not at all.
      There is no partial overlapping of two tetrahedra.
   3. Depending on the shape of the box, the mesh has 12, 16, or 24
      tetrahedra and 9, 10, or 12 vertices, respectively.
      3.1 The mesh of a cube has 12 tetrahedra. (MA has only one vertex
          in the cube's interior.)
      3.2 The mesh of a long box with two small square faces has 16
          tetrahedra. (MA has only two vertices in the box's interior
          forming a line segment.)
      3.3 The mesh of a shallow box with two big square faces has 24
          tetrahedra. (MA has four vertices in the box's interior forming a
          square.)
      3.4 The mesh of a box with three distinct dimensions also has 24
          tetrahedra. (MA has four vertices in the box's interior forming a
          rectangle.)

 @param[in] box
     The box shape specification (see drake::geometry::Box).
 @retval volume_mesh
 @tparam_nonsymbolic_scalar
 */
template <typename T>
VolumeMesh<T> MakeBoxVolumeMeshWithMa(const Box& box);

/*
 Generates a tetrahedral volume mesh of a given box by subdividing the box
 into _rectangular cells_ (volume bounded by six axis-aligned faces) and
 subdividing each rectangular cell into six tetrahedra. The output mesh will
 have these properties:

   1. The generated vertices are unique. There is no repeating vertices in
      the list of vertex coordinates.
   2. The generated tetrahedra are _conforming_. Two tetrahedra intersect in
      their shared face, or shared edge, or shared vertex, or not at all.
      There is no partial overlapping of two tetrahedra.

 @param[in] box
     The box shape specification (see drake::geometry::Box).
 @param[in] resolution_hint
     A measure (in meters) that controls the resolution of the mesh. The length
     of the axis-aligned edges of the mesh will be less than or equal to this
     parameter. The length of non-axis-aligned edges will be less than or equal
     to √3 of this parameter. The coarsest possible mesh can be made by
     providing a resolution hint at least as large as the box's largest
     dimension.
 @retval volume_mesh
 @tparam_nonsymbolic_scalar
 @note The mesh has no guarantee on the inner boundary for a rigid core.
 */
template <typename T>
VolumeMesh<T> MakeBoxVolumeMesh(const Box& box, double resolution_hint);

// TODO(DamrongGuoy): Consider advantage and disadvantage of other ways to
//  generate the surface mesh. Right now extracting surface mesh from volume
//  mesh is convenient but not the most efficient. Other ways to consider is
//  generating both the volume mesh and the surface mesh at the same time, or
//  generating the surface mesh directly without the volume mesh.
// TODO(DamrongGuoy): Document more precisely about the mesh pattern and its
//  edge length after we finalize the algorithms. Right now the algorithm for
//  the volume mesh is still evolving, so it's premature to promise a precise
//  outcome of the extracted surface mesh.
/*
 Generates a triangulated surface mesh of a given box. The output mesh will
 have these properties:
 1. The generated vertices are unique. There is no repeating vertices in the
    list of vertex coordinates.
 2. The generated triangles are _conforming_. Two triangles intersect in
    their shared edge, or shared vertices, or not at all. There is no partial
    overlapping of two triangles.
 @param[in] box
     The box shape specification (see drake::geometry::Box).
 @param[in] resolution_hint
     A measure (in meters) that controls the resolution of the mesh. Smaller
     values tends to give a denser mesh. The majority of the edges should have
     lengths approximately equal to this parameter. The coarsest possible mesh
     can be made by providing a resolution hint at least as large as the box's
     largest dimension.
 @retval surface_mesh
 @tparam T The underlying scalar type. Currently only supports double and
           AutoDiffXd.
 */
template <typename T>
TriangleSurfaceMesh<T> MakeBoxSurfaceMesh(const Box& box,
                                          double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.);
  // TODO(SeanCurtis-TRI): Consider putting an upper limit - as with the sphere.
  return ConvertVolumeToSurfaceMesh<T>(
      MakeBoxVolumeMesh<T>(box, resolution_hint));
}


/* The functions below are only support functions for the main API above. They
 are introduced here so that they can be put under test but shouldn't be
 otherwise considered as a viable public API. */

/*
 Calculates the sequential vertex index of the vertex specified by the
 (i, j, k)-index in the Cartesian grid defined by the number of vertices in
 x-, y-, and z-directions.
 @param i  Index in x-direction.
 @param j  Index in y-direction.
 @param k  Index in z-direction.
 @param num_vertices  Number of vertices in x-, y-, and z-directions.
 @pre 0 ≤ i < num_vertices.x(),
      0 ≤ j < num_vertices.y(), and
      0 ≤ k < num_vertices.z().
 */
int CalcSequentialIndex(int i, int j, int k, const Vector3<int>& num_vertices);

/*
 Generates unique vertices on a Cartesian grid of the box. In each of the
 x-, y-, and z-directions, the vertices are distributed uniformly.
 @param[in] box
     The box shape specification (see drake::geometry::Box).
 @param[in] num_vertices
     Number of vertices in each of x-, y-, and z-directions.
 @retval vertices
     The linear sequence of vertices consistent with CalcSequentialIndex.
 */
template <typename T>
std::vector<Vector3<T>> GenerateVertices(const Box& box,
                                         const Vector3<int>& num_vertices);

/*
 Adds six tetrahedra of a given rectangular cell to the list of tetrahedral
 elements. The rectangular cell is identified by the (i,j,k)-index of its
 lowest vertex. The (i,j,k)-indices of its 8 vertices are
 (i,j,k) + {0,1}x{0,1}x{0,1}.
 @param[in] lowest
     The (i,j,k) index of the lowest vertex of the rectangular cell.
 @param[in] num_vertices
     Number of vertices in each of x-, y-, and z-directions.
 @param[in,out] elements
     The six tetrahedra are added into this list of elements.
 */
void AddSixTetrahedraOfCell(const Vector3<int>& lowest,
                            const Vector3<int>& num_vertices,
                            std::vector<VolumeElement>* elements);

/*
 Generates connectivity for the tetrahedral elements of the mesh.
 @param[in] num_vertices
     Number of vertices in each of x-, y-, and z-directions.
 @return
     A sequence of tetrahedral elements that share unique vertices.
 */
std::vector<VolumeElement> GenerateElements(const Vector3<int>& num_vertices);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
