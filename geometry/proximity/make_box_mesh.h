#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

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
SurfaceMesh<T> MakeBoxSurfaceMesh(const Box& box, double resolution_hint) {
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
std::vector<VolumeVertex<T>> GenerateVertices(
    const Box& box, const Vector3<int>& num_vertices);

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
