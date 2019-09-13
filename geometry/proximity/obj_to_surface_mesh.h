#pragma once

#include <istream>
#include <string>
#include <vector>

#include <tiny_obj_loader.h>

#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

#ifndef DRAKE_DOXYGEN_CXX

/**
 Converts vertices of tinyobj to vertices of SurfaceMesh.
 @param tinyobj_vertices
     Vertices from tinyobj represented as `std::vector` of floating-point
     numbers.
 @param scale
     An optional scale to coordinates.
 @return
     Vertices for SurfaceMesh.
 @pre
     The size of `tinyobj_vertices` is divisible by three.
 */
std::vector<SurfaceVertex<double>> TinyObjToSurfaceVertices(
    const std::vector<tinyobj::real_t>& tinyobj_vertices, double scale);

/**
 Converts faces of tinyobj::mesh_t to faces of SurfaceMesh.
 @param mesh
     The mesh from tinyobj.
 @return
     The triangular faces for SurfaceMesh.
 @pre
     Every face is a triangle.
 */
std::vector<SurfaceFace> TinyObjToSurfaceFaces(const tinyobj::mesh_t& mesh);

#endif  // #ifndef DRAKE_DOXYGEN_CXX

// TODO(DamrongGuoy): Refactor the tinyobj usage between here and
//  ProximityEngine.

/**
 Constructs a surface mesh from a Wavefront .obj file and optionally scales
 coordinates by the given scale factor. Polygons will be triangulated if they
 are not triangles already.
 @param absolute_filename
     The file name with absolute path.
 @param scale
     An optional scale to coordinates.
 @throws std::runtime_error if the .obj file doesn't define a single object.
     This can happen if it is empty, if there are multiple object-name
     statements (e.g., "o object name"), or if there are faces defined
     outside a single object-name statement.
 @return surface mesh
 @note
     We only support an .obj file with only one object.
 */
SurfaceMesh<double> ReadObjToSurfaceMesh(const std::string& absolute_filename,
                                         double scale = 1.0);

/**
 Overload of @ref ReadObjToSurfaceMesh(const std::string&, double) with the
 Wavefront .obj file given in std::istream.
 */
SurfaceMesh<double> ReadObjToSurfaceMesh(std::istream* input_stream,
                                         double scale = 1.0);
}  // namespace internal
}  // namespace geometry
}  // namespace drake
