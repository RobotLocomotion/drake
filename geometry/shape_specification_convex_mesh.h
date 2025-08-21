#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/calc_signed_distance_to_surface_mesh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

/** Definition of a *convex* surface mesh.

 This shape is *not* the mesh contained in the file named by `filename`. It is
 the convex hull of that mesh. As such, the only contents of the mesh file that
 matter are the vertex positions. All other data is ignored. This includes
 materials, textures, normals etc. This is true for *all* roles. Its appearance
 in an illustration or perception role, will be a faceted polytope whose color
 is defined by its assigned properties (or by the geometry consumer's defaults).

 Because Drake computes the convex hull, the named mesh file need not be convex.

 The mesh is defined in a canonical frame C, implicit in the file parsed. Upon
 loading it in SceneGraph it can be scaled around the origin of C by a given
 `scale` amount. */
class Convex final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Convex);

  /** Constructs a convex shape specification from the file located at the
   given file path. Optionally uniformly scaled by the given scale factor.

   The mesh file referenced can be an .obj, .gltf, or a tetrahedral .vtk.

   @param filename     The file name; if it is not absolute, it will be
                       interpreted relative to the current working directory.
   @param scale        An optional scale to coordinates.

   @throws std::exception       if |scale| < 1e-8. Note that a negative scale is
                                considered valid. We want to preclude scales
                                near zero but recognise that scale is a
                                convenience tool for "tweaking" models. 8 orders
                                of magnitude should be plenty without
                                considering revisiting the model itself. */
  explicit Convex(const std::filesystem::path& filename, double scale = 1.0);

  /** File variant that allows for specification of non-uniform scale. */
  Convex(const std::filesystem::path& filename, const Vector3<double>& scale3);

  /** Constructs a convex shape specification from the contents of a
   Drake-supported mesh file type.

   The mesh is defined by the contents of a @ref supported_file_types
   "mesh file format supported by Drake". Those contents are passed in as
   `mesh_data`. For %Convex, the only supporting files required are those
   necessary to define vertex positions (e.g., a glTF's .bin file); material
   and texture files, if provided, are ignored and are therefore unnecessary.

   @param mesh_data   The in-memory file contents that define the vertex data
                      for this shape.
   @param scale       An optional scale to coordinates. */
  explicit Convex(InMemoryMesh mesh_data, double scale = 1.0);

  /** Mesh-contents variant that allows for specification of non-uniform scale.
   */
  Convex(InMemoryMesh mesh_data, const Vector3<double>& scale3);

  /** Constructs a convex shape specification from the given `source`.

   @param source   The source for the mesh data.
   @param scale    An optional scale to coordinates. */
  explicit Convex(MeshSource source, double scale = 1.0);

  /** Mesh-source variant that allows for specification of non-uniform scale.
   */
  Convex(MeshSource source, const Vector3<double>& scale3);

  /** Constructs an in-memory convex shape specification from the given points.

   The convex hull is computed from the points provided. The points are
   expected to be in the canonical frame of the shape. Optionally uniformly
   scaled by the given scale factor.

   @param points  The points whose convex hull define the shape.
   @param label   A label for the object. The label is used for warning and
                  error messages. Otherwise, the label has no other functional
                  purpose. It must consist of a single line.
   @param scale   An optional scale to coordinates.

   @throws std::exception       if label contains newlines.
   @throws std::exception       if |scale| < 1e-8. */
  Convex(const Eigen::Matrix3X<double>& points, const std::string& label,
         double scale = 1.0);

  /** Point variant that allows for specification of non-uniform scale. */
  Convex(const Eigen::Matrix3X<double>& points, const std::string& label,
         const Vector3<double>& scale3);

  ~Convex() final;

  /** Returns the source for this specification's mesh data. When working with
   %Convex, this API should only be used for introspection. The contract for
   %Convex is that the convex hull is always used in place of whatever
   underlying mesh declaration is provided. For all functional geometric
   usage, exclusively use the convex hull returned by GetConvexHull(). */
  const MeshSource& source() const { return source_; }

  /** Returns the extension of the underlying input mesh -- all lower case and
   including the dot. If `this` is constructed from a file path, the extension
   is extracted from the path. I.e., /foo/bar/mesh.obj and /foo/bar/mesh.OBJ
   would both report the ".obj" extension. The "extension" portion of the
   filename is defined as in std::filesystem::path::extension().

   If `this` is constructed using in-memory file contents, it is the extension
   of the MemoryFile passed to the constructor. */
  const std::string& extension() const { return source_.extension(); }

  /** Returns a single scale representing the _uniform_ scale factor.
   @throws if the scale is not uniform in all directions. */
  double scale() const;

  /** Returns general scale factors for this mesh. */
  const Vector3<double>& scale3() const { return scale_; }

  /** Reports the convex hull of the named mesh.

   Note: the convex hull is computed on demand on the first invocation. All
   subsequent invocations should have an O(1) cost.

   @throws if the referenced mesh data cannot be read or is degenerate
           (insufficient number of vertices, co-linear or coincident vertices,
           etc.) All of the vertices lying on a plane is *not* considered
           degenerate. */
  const PolygonSurfaceMesh<double>& GetConvexHull() const;

  const geometry::internal::Bvh<geometry::Obb,
                                geometry::TriangleSurfaceMesh<double>>&
  GetBVH() const;
  const geometry::TriangleSurfaceMesh<double>& GetSurfaceMesh() const;
  const geometry::internal::FeatureNormalSet& GetFeatureNormalSet() const;

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;
  VariantShapeConstPtr get_variant_this() const final;

  MeshSource source_;
  Vector3<double> scale_;
  // Allows the deferred computation of the hull on an otherwise const Convex.
  mutable std::shared_ptr<PolygonSurfaceMesh<double>> hull_{nullptr};

  // These members contain data structures that allow for fast searching
  // on the mesh. Are initialized only on demand and in general are nullptr
  mutable std::shared_ptr<geometry::TriangleSurfaceMesh<double>> tri_mesh_;
  mutable std::shared_ptr<geometry::internal::Bvh<
      geometry::Obb, geometry::TriangleSurfaceMesh<double>>>
      tri_bvh_{nullptr};
  mutable std::shared_ptr<geometry::internal::FeatureNormalSet>
      feature_normal_set_;
};

// TODO(DamrongGuoy): Update documentation when mesh is fully supported (i.e.,
// doesn't require equivocation here).
/** Definition of a general (possibly non-convex) mesh.

 The mesh may be a triangular surface mesh or a tetrahedral volume mesh,
 depending on how it used.

 Meshes can be used with all three roles but, currently, the support for the
 proximity role is limited. Where a general mesh is not supported, the mesh is
 replaced by its convex hull. See the documentation of QueryObject's proximity
 queries for more details. The notable cases where the actual mesh topology is
 used includes:

   - Computing signed distances from the %Mesh to query points (when it
     references a .obj file or a tetrahedral .vtk file).
   - Specifying the %Mesh as rigid hydroelastic (when it references a triangle
     .obj file or a tetrahedral .vtk file).
   - Specifying the %Mesh as compliant hydroelastic (when it references a
     tetrahedral .vtk file).
   - Specifying the %Mesh as deformable (when it references a tetrahedral .vtk
     file).

 This convex-hull substitution is a regrettable stop-gap solution until we fully
 support general, non-convex meshes throughout proximity queries.

 For visual roles (illustration and perception), the specified mesh file is
 used as directly as possible.

 The mesh is defined in a canonical frame C, implicit in the file parsed. Upon
 loading it in SceneGraph it can be scaled around the origin of C by a given
 `scale` amount.

 Note: a negative scale can be applied. This can be useful in mirroring the
 geometry (e.g., using a right hand mesh for a left hand). Mirroring the
 geometry will typically change the "winding" of the mesh elements. By
 convention, Drake looks at the _ordering_ of the vertices that form mesh
 elements (triangles and tetrahedra) and derives the notion of "inside" and
 "outside" relative to that element. In order to preserve the input mesh's
 definition of "inside" and "outside", when the mesh gets mirrored Drake may
 perturb the ordering of the vertex indices. For example, a triangle originally
 referencing vertices `[0 1 2]`, when mirrored may change to `[2 1 0]`, so don't
 be surprised if you introspect the details of the loaded mesh and you see such
 a change. An analogous change can affect the vertex ordering of tetrahedra in
 a volume mesh (i.e., a perturbation of the original vertex index list
 `[0 1 2 3]` to `[2 1 0 3]`). */
class Mesh final : public Shape {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Mesh);

  /** Constructs a mesh shape specification from the mesh file located at the
   given file path. Optionally uniformly scaled by the given scale factor.

   The mesh file referenced can be an .obj, a volume mesh in a .vtk, or a .gltf
   file. However, not all file formats are appropriate for all roles. (E.g.,
   a tetrahedral .vtk file should not be assigned a perception role.)

   @param filename     The file name; if it is not absolute, it will be
                       interpreted relative to the current working directory.
   @param scale        An optional scale to coordinates.

   @throws std::exception       if |scale| < 1e-8. Note that a negative scale is
                                considered valid. We want to preclude scales
                                near zero but recognise that scale is a
                                convenience tool for "tweaking" models. 8 orders
                                of magnitude should be plenty without
                                considering revisiting the model itself. */
  explicit Mesh(const std::filesystem::path& filename, double scale = 1.0);

  /** Mesh-file variant that allows for specification of non-uniform scale. */
  Mesh(const std::filesystem::path& filename, const Vector3<double>& scale3);

  /** Constructs a mesh shape specification from the contents of a
   Drake-supported mesh file type.

   The mesh is defined by the contents of a @ref supported_file_types
   "mesh file format supported by Drake". Those contents are passed in as
   `mesh_data`. The mesh data should include the main mesh file's contents as
   well as any supporting file contents as needed. See InMemoryMesh.

   @param mesh_data   The in-memory file contents that define the mesh data for
                      this shape.
   @param scale       An optional scale to coordinates. */
  explicit Mesh(InMemoryMesh mesh_data, double scale = 1.0);

  /** Mesh-contents variant that allows for specification of non-uniform scale.
   */
  Mesh(InMemoryMesh mesh_data, const Vector3<double>& scale3);

  /** Constructs a mesh shape specification from the given `source`.

   @param source   The source for the mesh data.
   @param scale    An optional scale to coordinates. */
  explicit Mesh(MeshSource source, double scale = 1.0);

  /** Mesh-source variant that allows for specification of non-uniform scale.
   */
  Mesh(MeshSource source, const Vector3<double>& scale3);

  ~Mesh() final;

  /** Returns the source for this specification's mesh data. */
  const MeshSource& source() const { return source_; }

  /** Returns the extension of the mesh type -- all lower case and including
   the dot. If `this` is constructed from a file path, the extension is
   extracted from the path. I.e., /foo/bar/mesh.obj and /foo/bar/mesh.OBJ would
   both report the ".obj" extension. The "extension" portion of the filename is
   defined as in std::filesystem::path::extension().

   If `this` is constructed using in-memory file contents, it is the extension
   of the MemoryFile passed to the constructor. */
  const std::string& extension() const { return source_.extension(); }

  /** Returns a single scale representing the _uniform_ scale factor.
   @throws if the scale is not uniform in all directions. */
  double scale() const;

  /** Returns general scale factors for this mesh.*/
  const Vector3<double>& scale3() const { return scale_; }

  /** Reports the convex hull of the named mesh.

   Note: the convex hull is computed on demand on the first invocation. All
   subsequent invocations should have an O(1) cost.

   @throws if the referenced mesh data cannot be read or is degenerate
           (insufficient number of vertices, co-linear or coincident vertices,
           etc.) All of the vertices lying on a plane is *not* considered
           degenerate. */
  const PolygonSurfaceMesh<double>& GetConvexHull() const;

  const geometry::internal::Bvh<geometry::Obb,
                                geometry::TriangleSurfaceMesh<double>>&
  GetBVH() const;
  const geometry::TriangleSurfaceMesh<double>& GetSurfaceMesh() const;
  const geometry::internal::FeatureNormalSet& GetFeatureNormalSet() const;

 private:
  void DoReify(ShapeReifier*, void*) const final;
  std::unique_ptr<Shape> DoClone() const final;
  std::string_view do_type_name() const final;
  std::string do_to_string() const final;
  VariantShapeConstPtr get_variant_this() const final;

  // NOTE: Cannot be const to support default copy/move semantics.
  MeshSource source_;
  Vector3<double> scale_;
  // Allows the deferred computation of the hull on an otherwise const Mesh.
  mutable std::shared_ptr<PolygonSurfaceMesh<double>> hull_{nullptr};
  mutable std::shared_ptr<geometry::TriangleSurfaceMesh<double>> tri_mesh_;
  mutable std::shared_ptr<geometry::internal::Bvh<
      geometry::Obb, geometry::TriangleSurfaceMesh<double>>>
      tri_bvh_{nullptr};
  mutable std::shared_ptr<geometry::internal::FeatureNormalSet>
      feature_normal_set_;
};

}  // namespace geometry
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::geometry, Convex, x, x.to_string())
DRAKE_FORMATTER_AS(, drake::geometry, Mesh, x, x.to_string())
