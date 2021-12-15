#pragma once

#include <algorithm>
#include <array>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/geometry/proximity/mesh_traits.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

// Forward declaration for friendship.
template <typename T>
class PolygonSurfaceMesh;

/** Representation of a polygonal face in a SurfacePolygon. */
class SurfacePolygon {
 public:
  // TODO(SeanCurtis-TRI): Consider making this copy-constructible, in which
  // case we can remove copy_to_unique() function, below.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SurfacePolygon)

  /** Returns the number of vertices in this face. */
  int num_vertices() const { return mesh_face_data_.at(index_); }

  /** Returns the vertex index in PolygonSurfaceMesh of the i-th vertex of
   this face.

   @param i  The local index of the vertex in this face.
   @pre 0 <= i < num_vertices() */
  int vertex(int i) const { return mesh_face_data_.at(index_ + 1 + i); }

  // TODO(SeanCurtis-TRI): Introduce vertices() method that returns a *range*
  //  iterator over the vertex indices referenced by this face.

  /** (Internal use only) Returns a copy of this, wrapped in a unique_ptr.
  This function is only intended for use by Drake's Python bindings. */
  std::unique_ptr<SurfacePolygon> copy_to_unique() const;

 private:
  /* Only PolygonSurfaceMesh can create faces. */
  template <typename>
  friend class PolygonSurfaceMesh;

  /* Constructs a SurfacePolygon.
   @param face_data  The mesh's face data from which this polygon is drawn. It
                     is the face data as documented in PolygonSurfaceMesh.
   @param index      The index into the face data of where this polygon's data
                     starts; the value contained is the number of vertices for
                     this polygon. It is _not_ the publicly visible index of the
                     polygon.
   */
  SurfacePolygon(const std::vector<int>* face_data, int index)
      : index_(index), mesh_face_data_(*face_data) {
    DRAKE_DEMAND(face_data != nullptr);
  }

  /* The index of *this* polygon in the polygonal surface. The index points to
   the first entry, which contains the vertex count. */
  const int index_{};

  /* The vertex data for the mesh to which this face belongs. */
  const std::vector<int>& mesh_face_data_;
};

/** %PolygonSurfaceMesh represents a surface comprised of *polygonal* elements
 (three or more sides).
 @tparam_nonsymbolic_scalar */
template <class T>
class PolygonSurfaceMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PolygonSurfaceMesh)

  /** @name Mesh type traits

   A collection of type traits to enable mesh consumers to be templated on mesh
   type. Each mesh type provides specific definitions of _element_ and
   _barycentric coordinates_. For %PolygonSurfaceMesh, an element is a
   polygon. */
  //@{

  using ScalarType = T;

  /** A definition that satisfies the interface that MeshFieldLinear requires.
   The value -1 indicates "unbounded" (as opposed to 3 for TriangleSurfaceMesh
   or 4 for VolumeMesh). It is up to MeshFieldLinear (and any other
   MeshType-compatible classes that pair with PolygonSurfaceMesh) to decide how
   they handle this unbounded value. */
  static constexpr int kVertexPerElement = -1;

  /** %PolygonSurfaceMesh doesn't actually support barycentric coordinates. This
   is here to satisfy the MeshFieldLinear interface. The dimension is selected
   to be zero to minimize memory footprint in case one is inadvertently
   instantiated. */
  template <typename U = T>
  using Barycentric = Vector<U, 0>;

  // TODO(SeanCurtis-TRI): Implement FaceRangeIterator faces() so people can
  //  iterate over the faces. And VertexRangeIterator vertices() so people can
  //  iterate over the vertices. This would help create better parallels with
  //  TriangleSurfaceMesh and VolumeMesh.

  /** Returns the polygonal element identified by the given index `e`.
    @pre e ∈ {0, 1, 2, ..., num_faces()-1}. */
  SurfacePolygon element(int e) const {
    DRAKE_DEMAND(0 <= e && e < num_faces());
    return {&face_data_, poly_indices_[e]};
  }

  /** Returns the vertex identified by the given index `v`.
   @pre v ∈ {0, 1, 2, ..., num_vertices()-1}. */
  const Vector3<T>& vertex(int v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
    return vertices_M_[v];
  }

  /** Returns the number of vertices in the mesh. */
  int num_vertices() const { return static_cast<int>(vertices_M_.size()); }

  /** Returns the number of elements in the mesh. For %PolygonSurfaceMesh, an
   element is a polygon. Returns the same number as num_faces() and enables
   mesh consumers to be templated on mesh type. */
  int num_elements() const { return num_faces(); }

  //@}

  /** Advanced() Constructs an *empty* mesh. This enables compatibility with STL
   container types and facilitates some unit tests. Otherwise, it shouldn't be
   used. */
  PolygonSurfaceMesh() : p_MSc_(Vector3<T>::Zero()) {}

  /** Constructs a mesh from specified vertex and mesh data.

   The vertices are simply a vector of position vectors (interpreted as being
   measured and expressed in the mesh's frame M).

   The polygon data is more complex. Syntactically, it is a sequence of integers
   which _encodes_ P polygons. Each polygon can have an arbitrary number of
   vertices. The encoding of the P polygons is as follows:

       |c₁|v₁₀|v₁₁|...|cᵢ|cᵢ₀|cᵢ₁|...|cₚ|cₚ₀|cₚ₁|...|

   Each polygon is defined in sequence. The definition consists of an integer
   indicating the *number* of vertices in that polygon (c₁, cᵢ, and cₘ in the
   illustration above). The next cᵢ integers in the sequence are zero-based
   indices into the vector of vertex positions (indicating which vertices the
   polygon spans). The vertex indices are sorted such that the plane normal
   found by applying the right-handed rule is used as the face normal.

   This implies the following:
     Polygon one:
       Located at index i₁ = 0 in `face_data`.
       c₁ = face_data[i₁] is the number of vertices in polygon one.
     Polygon two:
       Located at index i₂ = i₁ + c₁ + 1 in `face_data`.
       c₂ = face_data[i₂] is the number of vertices in polygon zero.
     Polygon j:
       Located at index iⱼ = iⱼ₋₁ + cⱼ₋₁ + 1
       cⱼ = face_data[iⱼ]

   The polygons must all be planar and convex.

   @param face_data  The sequence of counts and indices which encode the faces
                     of the mesh (see above).
   @param vertices   The vertex positions, measured and expressed in this
                     mesh's frame.
   @pre The indices in `face_data` all refer to valid indices into `vertices`.
   */
  PolygonSurfaceMesh(std::vector<int> face_data,
                     std::vector<Vector3<T>> vertices);

  /** Transforms the vertices of this mesh from its initial frame M to the new
   frame N. */
  void TransformVertices(const math::RigidTransform<T>& X_NM);

  // TODO(SeanCurtis-TRI): ContactResultsToLcm and HydroelasticContactInfo
  //  ostensibly support symbolic::Expression. However, the ContactSurface that
  //  they both interact with *doesn't*. Their unit tests blindly assume that
  //  there is full scalar support. ContactSurface calls ReverseFaceWinding so,
  //  for the offending unit tests to maintain the illusion of support, this
  //  must be defined in the header so they can compile and link -- although,
  //  the resulting ContactSurface<symbolic::Expression> is only a partial
  //  implementation and can't do any interesting math, this allows the tests
  //  to create the type they need. Ideally, the tests wouldn't be expressed
  //  in a way that suggests non-existent support is otherwise possible.
  //  Alternatively, there's a question about whether ContactSurface should be
  //  calling this method *at all*. Choosing that it's not necessary would
  //  likewise enable this implementation to move to the .cc file.
  /** Reverses the ordering of all the faces' indices. */
  void ReverseFaceWinding() {
    for (const int f_index : poly_indices_) {
      const int v_count = face_data_[f_index];
      /* The indices before and after the first and last entries.  */
      int f_0 = f_index;
      int f_n = f_index + v_count + 1;
      for (int i = 0; i < v_count / 2; ++i) {
        std::swap(face_data_[++f_0], face_data_[--f_n]);
      }
    }
    for (auto& n : face_normals_) {
      n = -n;
    }
  }

  /** Returns the number of polygonal elements in the mesh. */
  int num_faces() const { return static_cast<int>(poly_indices_.size()); }

  /** Returns area of a polygonal element.
   @pre f ∈ {0, 1, 2, ..., num_faces()-1}. */
  const T& area(int f) const {
    DRAKE_DEMAND(0 <= f && f < num_faces());
    return areas_[f];
  }

  /** Returns the total area of all the faces of this surface mesh. */
  const T& total_area() const { return total_area_; }

  /** Returns the unit face normal vector of a polygon. It respects the
   right-handed normal rule. A near-zero-area triangle may get an unreliable
   normal vector. A zero-area triangle will get a zero vector.
   @pre f ∈ {0, 1, 2, ..., num_faces()-1}. */
  const Vector3<T>& face_normal(int f) const {
    DRAKE_DEMAND(0 <= f && f < num_faces());
    return face_normals_[f];
  }

  /** Returns the geometric centroid of the element indicated be index `e`,
   measured and expressed in the mesh's frame M.
   @pre f ∈ {0, 1, 2, ..., num_faces()-1}. */
  const Vector3<T>& element_centroid(int e) const {
    DRAKE_DEMAND(0 <= e && e < num_faces());
    return element_centroid_M_[e];
  }

  /** Returns the geometric centroid of this mesh measured and expressed in
   the mesh's frame M. (M is the frame in which this mesh's vertices are
   measured and expressed.) Note that the centroid is not necessarily a point on
   the surface. If the total mesh area is exactly zero, we define the centroid
   to be (0,0,0). */
  const Vector3<T>& centroid() const { return p_MSc_; }

  /** See TriangleSurfaceMesh::CalcBaryCentric(). This implementation is
   provided to maintain compatibility with MeshFieldLinear. However, it only
   throws. %PolygonSurfaceMesh does not support barycentric coordinates. */
  template <typename C>
  Barycentric<promoted_numerical_t<T, C>> CalcBarycentric(
      const Vector3<C>& p_MQ, int p) const {
    unused(p_MQ, p);
    throw std::runtime_error(
        "PolygonSurfaceMesh::CalcBarycentric(): PolygonSurfaceMesh does not "
        "have barycentric coordinates.");
  }

  // TODO(DamrongGuoy): Consider using an oriented bounding box in obb.h.
  //  Currently we have a problem that PolygonSurfaceMesh and its vertices are
  //  templated on T, but Obb is for double only.
  /** Calculates the axis-aligned bounding box of this surface mesh M.
   @returns the center and the size vector of the box expressed in M's frame. */
  std::pair<Vector3<T>, Vector3<T>> CalcBoundingBox() const;

  // TODO(#12173): Consider NaN==NaN to be true in equality tests.
  /** Checks to see whether the given PolygonSurfaceMesh object is equal
   via deep exact comparison. NaNs are treated as not equal as per the IEEE
   standard.
   @param mesh The mesh for comparison.
   @returns `true` if the given mesh is equal. */
  bool Equal(const PolygonSurfaceMesh<T>& mesh) const;

  /* (Advanced) Provide access to the underlying face data to facilitate
   efficient visualization. */
  const std::vector<int>& face_data() const { return face_data_; }

  /** This is a stub method. It is provided so that PolygonSurfaceMesh provides
   a sufficient API to compile against MeshFieldLinear. However, we expect
   that the gradients of the field will always be provided when defining a
   MeshFieldLinear with a PolygonSurfaceMesh. Failure to provide those gradients
   will cause *this* method to be invoked which will, in turn, throw. */
  template <typename FieldValue>
  Vector3<FieldValue> CalcGradientVectorOfLinearField(
      const std::array<FieldValue, 3>& field_value, int p) const {
    unused(field_value, p);
    throw std::runtime_error(
        "PolygonSurfaceMesh::CalcGradientVectorOfLinearField(): "
        "PolygonSurfaceMesh does not support this calculation. Defining a "
        "MeshFieldLinear on a PolygonSurfaceMesh requires field gradients to "
        "be provided at construction.");
  }

 private:
  /* Calculates the area and face normal of a polygon. Further computes its
   contribution to the surface centroid.

   @param poly_index The index of the polygon to compute the derived quantities.
                     Must be in the range [0, poly_indices_.size()). */
  void CalcAreaNormalAndCentroid(int poly_index);

  /* The encoding of the mesh's polygons. See the advanced constructor for
   details. */
  std::vector<int> face_data_;

  /* The ith polygon is defined in face_data_ at index poly_indices_[i]. In
   other words, this contains the indices of face_data_ that point to the c₁,
   cᵢ, ..., cₘ entries for M polygons in the mesh. */
  std::vector<int> poly_indices_;

  /* The vertices referenced by the mesh's polygons. */
  std::vector<Vector3<T>> vertices_M_;

  /* Derived quantities of the mesh -- computed as elements are added. */

  /* Per-polygon areas. areas_.size() == poly_indices_.size() is always true. */
  std::vector<T> areas_;

  /* The total area of all polygons in the mesh. */
  T total_area_{0};

  /* Per-polygon normals. face_normals_.size() == poly_indices_.size() is always
   true. */
  std::vector<Vector3<T>> face_normals_;

  /* Per-polygon centroids. */
  std::vector<Vector3<T>> element_centroid_M_;

  /* The geometric centroid Sc of the surface mesh, measured and expressed in
   the mesh frame M. */
  Vector3<T> p_MSc_;
};

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class PolygonSurfaceMesh)

}  // namespace geometry
}  // namespace drake
