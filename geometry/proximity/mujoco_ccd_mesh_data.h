#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Precomputed collision data for one convex mesh, in the format consumed by
 MuJoCo's native convex collision detection (GJK/EPA with multi-point contact
 recovery by contact-polygon clipping).

 The polygon tables replicate what MuJoCo's model compiler produces in
 mjCMesh::MakePolygons(): the (near-)coplanar faces of the convex hull are
 merged into maximal polygons, and a vertex-to-polygon adjacency map is
 provided so that the collision code can identify the mesh feature (face,
 edge, or vertex) participating in a contact. Field names follow MuJoCo's
 mjCCDObj mesh data (see mjModel's mesh_poly* arrays) to keep the
 correspondence auditable, with all addresses local to this mesh (i.e.,
 zero-offset).

 Vertices are stored in single precision because that is how MuJoCo stores
 mesh vertices; using the same representation keeps the contact results
 bit-comparable with MuJoCo's. */
struct MujocoCcdMeshData {
  /* Hull vertex positions, measured and expressed in the mesh's canonical
   frame; 3 * nvert floats. */
  std::vector<float> vert;
  int nvert{0};

  /* The mean of the hull vertices, used to seed the collision query (MuJoCo
   seeds with the geom center). It is strictly inside the hull. */
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};

  /* Number of merged polygons. */
  int polynum{0};
  /* Outward unit normal of each polygon (3 * polynum), computed from the
   polygon's first three vertices exactly as MuJoCo's
   mjCMesh::MakePolygonNormals() does. */
  std::vector<double> polynormal;
  /* Start index into `polyvert` for each polygon (size polynum). */
  std::vector<int> polyvertadr;
  /* Number of vertices of each polygon (size polynum). */
  std::vector<int> polyvertnum;
  /* Concatenated vertex indices of each polygon, wound counterclockwise when
   viewed from outside the hull. */
  std::vector<int> polyvert;

  /* Start index into `polymap` for each vertex (size nvert). */
  std::vector<int> polymapadr;
  /* Number of adjacent polygons of each vertex (size nvert). A vertex that
   ended up interior to a merged polygon has count zero. */
  std::vector<int> polymapnum;
  /* Concatenated adjacent-polygon indices of each vertex. */
  std::vector<int> polymap;

  /* Maximum number of vertices in any polygon; sizes MuJoCo's clipping
   scratch buffers (MuJoCo's per-model `npolygonmax`). */
  int npolygonmax{0};
  /* Maximum number of polygons adjacent to any vertex (MuJoCo's per-model
   `nmeshdegmax`). */
  int nmeshdegmax{0};
};

/* Builds the MuJoCo collision data for the given convex hull, replicating the
 preprocessing MuJoCo's model compiler performs in mjCMesh::MakePolygons():
 hull faces are bucketed by their outward normal direction quantized to
 0.01-radian bins, faces in the same bucket that share (directed) edges are
 merged, and each merged region's boundary is traced into a single polygon.

 @param hull  A convex polygonal surface mesh, e.g. the result of
              MakeConvexHull(). Faces that are already merged (e.g. by
              qhull) are handled fine; the quantized-normal merge is applied
              on top. Faces are normally wound counterclockwise viewed from
              the outside; a uniformly inward-wound mesh (e.g. cached hull
              topology re-instantiated at a mirroring scale) is detected via
              its negative enclosed volume and handled by reversing the
              loops.
 @pre hull is a closed convex surface with consistently oriented faces. */
MujocoCcdMeshData MakeMujocoCcdMeshData(const PolygonSurfaceMesh<double>& hull);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
