#pragma once

/** @file
 There are multiple ways to compute a contact surface depending on the geometry
 representations and compliance types involved. However, they should all produce
 ContactSurface instances that satisfy some basic invariants. These functions
 assist in maintaining those invariants.
 */

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh_field.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

/* @name "MeshBuilder" implementations

 These MeshBuilder classes are used as the function template parameter in
 various contact surface algorithms. They provide sufficient infrastructure to
 build meshes with different representations (e.g., triangle vs polygon).

 MeshBuilders provide two services:

   - Collect mesh and field data associated with the polygon that arises from
     tet-tet, tet-tri, tet-plane, tri-half space, etc. intersections.
   - Compute the mesh and field type tailored to that builder.

 A MeshBuilder should be thought of as a frame-dependent quantity. The mesh it
 builds is likewise a frame-dependent quantity. As such, it should be named with
 the expected frame clearly identified: e.g., builder_W. Please note the frame
 expectations on the various function parameters. In the classes' documentation,
 we refer to the builder's frame as B. Some of the APIs require the fields or
 meshes that are colliding, and their frames will be named in the scope of those
 functions. */
//@{

/* A MeshBuilder type to build a triangle surface mesh. The mesh is built
 incrementally. Vertices get added to the mesh (each with a corresponding
 pressure field value). Subsequently, the polygon is declared, referencing
 previously added vertices by index.

 The TriMeshBuilder will always tessellate every polygon around its centroid
 (adding an additional vertex to the declared vertices). */
template <typename T>
class TriMeshBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TriMeshBuilder);

  using ScalarType = T;

  TriMeshBuilder() = default;

  /* Adds a vertex V (and its corresponding field value) to the mesh.

   @param p_BV         The position of the new vertex (measured and expressed in
                       the builder's frame B).
   @param field_value  The value of the pressure field evaluated at V.
   @returns The index of the newly added vertex. */
  int AddVertex(const Vector3<T>& p_BV, const T& field_value) {
    vertices_B_.push_back(p_BV);
    pressures_.push_back(field_value);
    return static_cast<int>(vertices_B_.size() - 1);
  }

  /* Adds the polygon to the in-progress mesh. The polygon is defined by
   indices into the set of vertices that have already been added to the builder.

   @param polygon_vertices  The definition of the polygon to add, expressed as
                            ordered indices into the currently existing
                            vertices. They should be ordered in a counter-
                            clockwise manner such that the implied face normal
                            points "outward" (using the right-hand rule).
   @param nhat_B            The normal to the polygon, measured and expressed in
                            the builder's frame B.
   @param grad_e_MN_B       The gradient of the pressure field in the domain of
                            the polygon, expressed in the builder's frame B.
   @returns The number of faces added to the mesh.

   @sa AddVertex(). */
  int AddPolygon(const std::vector<int>& polygon_vertices,
                 const Vector3<T>& nhat_B, const Vector3<T>& grad_e_MN_B);

  /* Returns the total number of vertices accumulated so far. */
  int num_vertices() const { return static_cast<int>(vertices_B_.size()); }

  /* Returns the total number of faces added by calls to AddPolygon(). */
  int num_faces() const { return static_cast<int>(faces_.size()); }

  /* Create a mesh and field from the mesh data that has been aggregated by
   this builder. */
  std::pair<std::unique_ptr<TriangleSurfaceMesh<T>>,
            std::unique_ptr<TriangleSurfaceMeshFieldLinear<T, T>>>
  MakeMeshAndField();

 private:
  /* The faces of the mesh being built. */
  std::vector<SurfaceTriangle> faces_;
  /* The vertices of the mesh being built. */
  std::vector<Vector3<T>> vertices_B_;
  /* The pressure values (e) of the surface being built. */
  std::vector<T> pressures_;
};

/* A MeshBuilder type to build a polygon surface mesh. The mesh is built
 incrementally. Vertices get added to the mesh (each with a corresponding
 pressure field value). Subsequently, the polygon is declared, referencing
 previously added vertices by index. */
template <typename T>
class PolyMeshBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PolyMeshBuilder);

  using ScalarType = T;

  PolyMeshBuilder();

  /* Adds a vertex V (and its corresponding field value) to the mesh.

   @param p_BV         The position of the new vertex (measured and expressed in
                       the builder's frame B).
   @param field_value  The value of the pressure field evaluated at V.
   @returns The index of the newly added vertex. */
  int AddVertex(const Vector3<T>& p_BV, const T& field_value) {
    vertices_B_.push_back(p_BV);
    pressures_.push_back(field_value);
    return static_cast<int>(vertices_B_.size() - 1);
  }

  /* Adds the polygon to the in-progress mesh. The polygon is defined by
   indices into the set of vertices that have already been added to the builder.

   @param polygon_vertices  The definition of the polygon to add, expressed as
                            ordered indices into the currently existing
                            vertices. They should be ordered in a counter-
                            clockwise manner such that the implied face normal
                            points "outward" (using the right-hand rule).
   @param nhat_B            The normal to the polygon, measured and expressed in
                            the builder's frame B.
   @param grad_e_MN_B       The gradient of the pressure field in the domain of
                            the polygon, expressed in the builder's frame B.
   @returns The number of faces added to the mesh.

   @sa AddVertex(). */
  int AddPolygon(const std::vector<int>& polygon_vertices,
                 const Vector3<T>& nhat_B, const Vector3<T>& grad_e_MN_B);

  /* Returns the total number of vertices accumulated so far. */
  int num_vertices() const { return static_cast<int>(vertices_B_.size()); }

  /* Returns the total number of faces added by calls to AddPolygon(). */
  int num_faces() const { return polygon_count_; }

  /* Create a mesh and field from the mesh data that has been aggregated by
   this builder. */
  std::pair<std::unique_ptr<PolygonSurfaceMesh<T>>,
            std::unique_ptr<PolygonSurfaceMeshFieldLinear<T, T>>>
  MakeMeshAndField();

  /* Expose the accumulated, per-face gradients for testing. */
  std::vector<Vector3<T>>& mutable_per_element_gradients() {
    return grad_e_MN_B_per_face_;
  }

 private:
  /* The number of polygons that have been added. It can't simply be inferred
   from face_data_.size() because of the face encoding. */
  int polygon_count_{0};
  /* The definition of all faces of the mesh being built. */
  std::vector<int> face_data_;
  /* The per-face gradients of the pressure field. */
  std::vector<Vector3<T>> grad_e_MN_B_per_face_;
  /* The vertices of the mesh being built. */
  std::vector<Vector3<T>> vertices_B_;
  /* The pressure values (e) of the surface being built. */
  std::vector<T> pressures_;
};

/* Given a planar, N-sided convex `polygon`, computes its centroid. The
 `polygon` is represented as an ordered list of indices into the given set of
 `vertices_F`. The resulting centroid will be measured and expressed in the same
 Frame F as the provided vertices.

 In debug builds, this method will do _expensive_ validation of its parameters.

 @param polygon
     The planar N-sided convex polygon defined by three or more ordered indices
     into `vertices_F`.
 @param[in] n_F
     A vector that is perpendicular to the `polygon`'s plane, expressed in
     Frame F.
 @param vertices_F
     The set of vertices from which the polygon is defined, each measured and
     expressed in Frame F.
 @retval p_FC, the position of the polygon's centroid C, measured and expressed
     in Frame F.
 @pre `polygon.size()` >= 3.
 @pre `n_F` is  perpendicular to the defined `polygon`'s plane.
 @pre `n_F` has non-trivial length.
 @pre `polygon` is planar.
 @tparam_nonsymbolic_scalar
 */
template <typename T>
Vector3<T> CalcPolygonCentroid(const std::vector<int>& polygon,
                               const Vector3<T>& n_F,
                               const std::vector<Vector3<T>>& vertices_F);

// TODO(SeanCurtis-TRI): Consider creating an overload of this that *computes*
//  the normal and then invokes this one for contexts where they don't have the
//  normal convenient.

/* Adds the planar, N-sided convex `polygon` to the given set of `faces` and
 `vertices` as a set of N triangles. A new vertex is introduced at the
 `polygon`'s centroid and one triangle is added for each edge, formed by the
 edge and the centroid vertex.

 In debug builds, this method will do _expensive_ validation of its parameters.

 @param[in] polygon
     The input polygon is represented by three or more ordered indices into
     `vertices_F`. This polygon is _not_ in `faces` and will not, itself, appear
     in `faces` when done.
 @param[in] n_F
     The vector that is perpendicular to the `polygon`, expressed in frame F.
 @param[in, out] faces
     New triangles are added into `faces`. Each new triangle has the same
     orientation (same normal vector) as the input polygon.
 @param[in ,out] vertices_F
     The set of vertex positions to be extended, each vertex is measured and
     expressed in frame F. It is assumed that `polygon`'s indices all reference
     vertices in this vector. One vertex will be added -- the polygon's
     centroid.
 @pre `faces` and `vertices_F` are not `nullptr`.
 @pre `polygon.size()` >= 3.
 @pre Each index in `polygon` indexes a valid vertex in `vertices_F`.
 @pre `polygon` is planar.
 @pre `n_F` is perpendicular to the defined `polygon`.
 @pre `n_F` has non-trivial length.
 @tparam_nonsymbolic_scalar */
template <typename T>
void AddPolygonToTriangleMeshData(const std::vector<int>& polygon,
                                  const Vector3<T>& n_F,
                                  std::vector<SurfaceTriangle>* faces,
                                  std::vector<Vector3<T>>* vertices_F);

/* Adds a polygon (defined by indices into a set of vertices) into the polygon
 face data (as defined by PolygonSurfaceMesh).
 
 This is similar to AddPolygonToTriangleMeshData() in that the specified polygon
 is added to some representation of mesh faces. It's different in the following
 ways:

   1. The face_data isn't literally a vector of discrete faces, but an encoding
      of the entire set of mesh polygons (as documented by PolygonSurfaceMesh).
   2. No normal or vertices are required because adding a polygon requires no
      operation on pre-existing vertex data (i.e., calculation of centroid). */
void AddPolygonToPolygonMeshData(const std::vector<int>& polygon,
                                 std::vector<int>* face_data);

/* Determines if the indicated triangle has a face normal that is "in the
 direction" of the given normal.

 The definition of "in the direction" is within a hard-coded tolerance 5π/8,
 which was empirically determined. Note that there is no one value that always
 works (see documentation of IsFaceNormalAlongPressureGradient() in
 mesh_intersection.h for examples).

 @param normal_F    The normal to test against, expressed in Frame F.
 @param surface_M   The mesh from which the triangle is drawn, measured and
                    expressed in Frame M.
 @param tri_index   The index of the triangle in `surface_M` to test.
 @param R_FM        The relative orientation between the bases of M and F.
 @pre `normal_F` is unit length.
 @return `true` if the angle between `normal_F` and the triangle normal lies
          within the hard-coded tolerance.
 @tparam_nonsymbolic_scalar */
template <typename T>
bool IsFaceNormalInNormalDirection(const Vector3<T>& normal_F,
                                   const TriangleSurfaceMesh<T>& surface_M,
                                   int tri_index,
                                   const math::RotationMatrix<T>& R_FM);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
