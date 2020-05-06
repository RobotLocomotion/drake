#include "drake/geometry/mesh_painter_system.h"

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/shape_specification.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace geometry {

using Eigen::Vector2d;
using Eigen::Vector3d;
using internal::BoundingVolumeHierarchy;
using internal::BvttCallbackResult;
using internal::PosedHalfSpace;
using math::RigidTransformd;
using std::make_pair;
using std::make_unique;
using std::map;
using std::move;
using std::pair;
using std::tie;
using std::unique_ptr;
using std::vector;
using systems::Context;
using systems::InputPort;
using systems::LeafSystem;
using systems::OutputPort;
using systems::sensors::ImageRgba8U;
using systems::sensors::PixelType;

namespace {
// The representation of the patch on a canvas. It is the surface mesh that
// spans the intersection between the surface canvas mesh and the volume painter
// mesh with the accompanying vector of per-vertex texture coordinates.
struct PaintedPatch {
  unique_ptr<SurfaceMesh<double>> mesh;
  vector<Vector2d> uvs;
};

/* Given a polygon defined by an ordered sequence of vertex positions, defines
 a new polygon from the minimum number of those positions. This eliminates
 duplicate vertex positions. In other words, for a sequence `A,B,B,C,A` with
 indices (0, 1, 2, 3, 4), the  pair of B's is reduced to one B and the first and
 last A vertices are considered duplicates and the result would be e.g.,
 (0, 1, 3) (although (0, 2, 3), (1, 3, 4), (2, 3, 4) would all be considered
 equivalent). The polygon might be reduced to a pair of points (i.e., `A,A,B,B`
 becomes (0, 2)) or a single point (`A,A,A` becomes (0,)).
 @param[in] polygon
     The input polygon, pass by value.
 @return
     The indices of the subset of vertex positions that spans the same polygon.
 */
template <typename T>
vector<int> ComputeMinimumPolygon(std::vector<Vector3<T>> polygon) {
  // TODO(SeanCurtis-TRI): The resulting polygon depends on the order of the
  //  inputs. Imagine I have vertices A, A', A'' (such that |X - X'| < eps.
  //  The sequence AA'A'' would be reduced to AA''
  //  The sequence A'A''A would be reduced to A'.
  //  The sequence A''AA' would be reduced to A''A.
  //  In all three cases, the exact same polygon is defined on input, but the
  //  output is different. This should be documented and/or fixed.
  if (polygon.size() <= 1) return {};

  auto near = [](const Vector3<T>& p, const Vector3<T>& q) -> bool {
    // TODO(SeanCurtis-TRI): This represents 5-6 bits of loss. Confirm that a
    //  tighter epsilon can't be used. This should probably be a function of the
    //  longest edge involved.
    // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
    // too small, especially when the objects are not axis-aligned.
    const double kEpsSquared(1e-14 * 1e-14);
    return (p - q).squaredNorm() < kEpsSquared;
  };

  vector<int> indices;
  indices.reserve(polygon.size());

  const int v_count = static_cast<int>(polygon.size());
  Vector3<T>* last_value = &polygon[v_count - 1];
  for (int b = 0; b < v_count; ++b) {
    if (!near(*last_value, polygon[b])) {
      indices.push_back(b);
      last_value = &polygon[b];
    }
  }

  return indices;
}

/* Calculates the intersection point between an infinite straight line spanning
 points A and B and the bounding plane of the half space H. Represents it as
 a set of weights (α, β) such that the intersecting point P = αA + βB.
 @param p_FA
     Point A measured and expressed in the common frame F.
 @param p_FB
     Point B measured and expressed in the common frame F.
 @param H_F
     The half space H measured and expressed in frame F (i.e., points also
     measured and expressed in frame F can be tested against it).
 @pre
     1. Points A and B are not coincident.
     2. One of A and B is outside the half space (and the other is contained in
        the half space).
     3. The line is _not_ parallel with the half space. Given previous
        requirements, this implies that they cannot both lie *on* the boundary
        of the half space.
 @retval (α, β) the two weights to define the intersection point. α + β = 1
 */
template <typename T>
Vector2<T> CalcIntersection(const Vector3<T>& p_FA, const Vector3<T>& p_FB,
                            const PosedHalfSpace<T>& H_F) {
  const T a = H_F.CalcSignedDistance(p_FA);
  const T b = H_F.CalcSignedDistance(p_FB);
  // We require that A and B classify in opposite directions (one inside and one
  // outside). Outside has a strictly positive distance, inside is non-positive.
  // We confirm that their product is non-positive and that at least one of the
  // values is positive -- they can't both be zero. This prevents b - a becoming
  // zero and the corresponding division by zero.
  DRAKE_ASSERT(a * b <= 0 && (a > 0 || b > 0));
  const T wa = b / (b - a);
  const T wb = T(1.0) - wa;  // Enforce a + b = 1.
  return {wa, wb};
}

/* Intersects a polygon with the half space H. It keeps the part of
 the polygon contained in the half space (signed distance is <= 0).
 The half space `H_F` and vertex positions of `polygon_vertices_F` are both
 defined in a common frame F.
 @param[in] polygon_vertices_F
     Input polygon is represented as a sequence of positions of its vertices.
     The input polygon is allowed to have zero area.
 @param[in] uvs
     The per vertex texture coordinates.
 @param[in] H_F
     The clipping half space H in frame F.
 @return
     Output polygon is represented as a sequence of positions of its vertices
     with its corresponding texture coordinates.
     It could be an empty sequence if the input polygon is entirely outside
     the half space. It could be the same as the input polygon if the input
     polygon is entirely inside the half space. The output polygon is guaranteed
     to be planar (within floating point tolerance) and, if the polygon has
     area, the normal implied by the winding will be the same as the input
     polygon.
 @pre `polygon_vertices_F` has at least three vertices.
 @pre `polygon_vertices_F.size() = uvs.size()`.
 @pre the vertices in `polygon_vertices_F` are all planar.
 @note
     1. For an input polygon P that is parallel to the plane of the half space,
        there are three cases:
        1.1 If P is completely inside the half space, the output polygon
            will be the same as P.
        1.2 If P is completely outside the half space, the output polygon will
            be empty.
        1.3 If P is on the plane of the half space, the output polygon will be
           the same as P.
     2. For an input polygon P outside the half space with one edge on the
        plane of the half space, the output polygon will be a zero-area
        4-gon with two pairs of duplicate vertices.
     3. For an input polygon P outside the half space with one vertex on the
        plane of the half space, the output polygon will be a zero-area
        triangle with three duplicate vertices.
*/
template <typename T>
pair<vector<Vector3<T>>, vector<Vector2<T>>> ClipPolygonByHalfSpace(
    const vector<Vector3<T>>& polygon_vertices_F, const vector<Vector2<T>>& uvs,
    const PosedHalfSpace<T>& H_F) {
  DRAKE_DEMAND(polygon_vertices_F.size() == uvs.size());
  // Note: this is the inner loop of a modified Sutherland-Hodgman algorithm for
  // clipping a polygon.
  vector<Vector3<T>> output_vertices_F;
  vector<Vector2<T>> output_uvs;
  // Note: This code is correct for size < 3, but pointless so we make no effort
  // to support it or test it.
  const int size = static_cast<int>(polygon_vertices_F.size());

  // TODO(SeanCurtis-TRI): If necessary, this can be made more efficient:
  //  eliminating the modulus and eliminating the redundant "inside" calculation
  //  on previous (by pre-determining previous and its "containedness" and then
  //  propagating current -> previous in each loop. Probably a desirable
  //  optimization as we need to make all of this work as cheap as possible.
  for (int i = 0; i < size; ++i) {
    const int prev_index = (i - 1 + size) % size;
    const Vector3<T>& current = polygon_vertices_F[i];
    const Vector3<T>& previous = polygon_vertices_F[prev_index];
    const bool current_contained = H_F.CalcSignedDistance(current) <= 0;
    const bool previous_contained = H_F.CalcSignedDistance(previous) <= 0;
    if (current_contained) {
      if (!previous_contained) {
        // Current is inside and previous is outside. Compute the point where
        // that edge enters the half space. This is a new vertex in the clipped
        // polygon and must be included before current.
        const Vector2<T>& w = CalcIntersection(current, previous, H_F);
        output_vertices_F.emplace_back(w[0] * current + w[1] * previous);
        output_uvs.emplace_back(w[0] * uvs[i] + w[1] * uvs[prev_index]);
      }
      output_vertices_F.push_back(current);
      output_uvs.push_back(uvs[i]);
    } else if (previous_contained) {
      // Current is outside and previous is inside. Compute the point where
      // the edge exits the half space. This is a new vertex in the clipped
      // polygon and is included *instead* of current.
      const Vector2<T>& w = CalcIntersection(current, previous, H_F);
      output_vertices_F.emplace_back(w[0] * current + w[1] * previous);
      output_uvs.emplace_back(w[0] * uvs[i] + w[1] * uvs[prev_index]);
    }
  }
  return {output_vertices_F, output_uvs};
}

/* Intersects a triangle with a tetrahedron, returning the portion of the
 triangle with non-zero area contained in the tetrahedron.
 @param element
     Index of the tetrahedron in a volume mesh.
 @param volume_M
     The volume mesh whose vertex positions are expressed in M's frame.
 @param face
     Index of the triangle in a surface mesh.
 @param surface_N
     The surface mesh whose vertex positions are expressed in N's frame.
 @param surface_uvs
     The per-vertex texture coordinates for `surface_N`. The ith vertex in
     `surface_N` has texture coordinate `surface_uvs[i]`.
 @param X_MN
     The pose of the surface frame N in the volume frame M.
 @retval polygon_M
     The output polygon represented by a sequence of positions of its
     vertices, expressed in M's frame. The nature of triangle-tetrahedron
     intersection means that this polygon can have up to seven vertices (i.e.,
     if the plane of the triangle cuts the tetrahedron into a rectangle, and
     the a vertex of the rectangle lies inside the triangle).
 @note
     1. If the triangle is outside the tetrahedron with one vertex on a
        face of the tetrahedron, the output polygon will be empty.
     2. If the triangle is outside the tetrahedron with an edge on a face
        of the tetrahedron, the output polygon will be empty.
     3. If the triangle lies on the plane of a tetrahedron face, the output
        polygon will be that part of the triangle inside the face of the
        tetrahedron (non-zero area restriction still applies).
 */
vector<Vector2d> ClipTriangleByTetrahedronForUvPatch(
    VolumeElementIndex element, const VolumeMesh<double>& volume_M,
    SurfaceFaceIndex face, const SurfaceMesh<double>& surface_N,
    const vector<Vector2d>& surface_uvs,
    const math::RigidTransform<double>& X_MN) {
  // Initialize output polygon in M's frame from the triangular `face` of
  // surface_N.
  // TODO(SeanCurtis-TRI): Consider using a simple array-like object to avoid
  //  allocation. Will require additional "size" parameter and possibly have
  //  to change the return type.
  vector<Vector3d> polygon_M;
  vector<Vector2d> uvs;
  polygon_M.reserve(7);
  uvs.reserve(7);
  for (int i = 0; i < 3; ++i) {
    SurfaceVertexIndex v = surface_N.element(face).vertex(i);
    // TODO(SeanCurtis-TRI): The `M` in `r_MV()` is different from the M in this
    //  function. More evidence that the `vertex(v).r_MV()` notation is *bad*.
    const Vector3d& p_NV = surface_N.vertex(v).r_MV();
    polygon_M.emplace_back(X_MN * p_NV);
    uvs.emplace_back(surface_uvs[v]);
  }
  // Get the positions, in M's frame, of the four vertices of the tetrahedral
  // `element` of volume_M.
  Vector3d p_MVs[4];
  for (int i = 0; i < 4; ++i) {
    VolumeVertexIndex v = volume_M.element(element).vertex(i);
    p_MVs[i] = volume_M.vertex(v).r_MV();
  }
  // Sets up the four half spaces associated with the four triangular faces of
  // the tetrahedron. Assume the tetrahedron has the fourth vertex seeing the
  // first three vertices in CCW order; for example, a tetrahedron of (Zero(),
  // UnitX(), UnitY(), UnitZ()) (see the picture below) has this orientation.
  //
  //      +Z
  //       |
  //       v3
  //       |
  //       |
  //     v0+------v2---+Y
  //      /
  //     /
  //   v1
  //   /
  // +X
  //
  // This table encodes the four triangular faces of the tetrahedron in such
  // a way that each right-handed face normal points outward from the
  // tetrahedron, which is suitable for setting up the half space. Refer to
  // the above picture.
  const int faces[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
  for (auto& face_vertex : faces) {
    const Vector3d& p_MA = p_MVs[face_vertex[0]];
    const Vector3d& p_MB = p_MVs[face_vertex[1]];
    const Vector3d& p_MC = p_MVs[face_vertex[2]];
    // We'll allow the PosedHalfSpace to normalize our vector.
    const Vector3d normal_M = (p_MB - p_MA).cross(p_MC - p_MA);
    PosedHalfSpace<double> half_space_M(normal_M, p_MA);
    // Intersects the output polygon by the half space of each face of the
    // tetrahedron.
    tie(polygon_M, uvs) = ClipPolygonByHalfSpace(polygon_M, uvs, half_space_M);
    DRAKE_DEMAND(polygon_M.size() == uvs.size());
  }

  // If the intersection reduced it to something with no area; stop.
  if (polygon_M.size() < 3) return {};

  // TODO(DamrongGuoy): Remove the code below when ClipPolygonByHalfSpace()
  //  stops generating duplicate vertices. See the note in
  //  ClipPolygonByHalfSpace().

  // Remove possible duplicate vertices from ClipPolygonByHalfSpace().
  vector<int> required_indices = ComputeMinimumPolygon(polygon_M);
  if (required_indices.size() < 3) return {};
  if (required_indices.size() != polygon_M.size()) {
    // We've removed some duplicates -- extract uvs.
    vector<Vector2d> new_uvs;
    for (int index : required_indices) new_uvs.push_back(uvs[index]);
    swap(uvs, new_uvs);
  }

  return uvs;
}

/* Computes the patch on the canvas that is currently painted by the given
 painter.

 @param canvas_C  The canvas object measured and expressed in frame C.
 @param X_WC      The relative pose between C and a common frame W.
 @param painter_P The painter object measured and expressed in frame P.
 @param X_WP      The relative pose between P and a common frame W.
 */
PaintedPatch CalculatePaintedPatch(const TexturedCanvasMesh& canvas_C,
                                   const RigidTransformd& X_WC,
                                   const PainterMesh& painter_P,
                                   const RigidTransformd& X_WP) {
  vector<SurfaceFace> surface_faces;
  vector<SurfaceVertex<double>> surface_vertices_P;
  vector<Vector2d> surface_uvs;
  const RigidTransformd X_PC = X_WP.inverse() * X_WC;
  auto callback = [&volume_P = *painter_P.mesh, &surface_C = *canvas_C.mesh,
                   &canvas_uvs = canvas_C.uvs, &X_PC, &surface_faces,
                   &surface_uvs,
                   &surface_vertices_P](VolumeElementIndex tet_index,
                                        SurfaceFaceIndex tri_index) {
    vector<Vector2d> polygon_uvs = ClipTriangleByTetrahedronForUvPatch(
        tet_index, volume_P, tri_index, surface_C, canvas_uvs, X_PC);

    const int num_vertices = static_cast<int>(polygon_uvs.size());

    if (num_vertices == 0) return BvttCallbackResult::Continue;

    // Build a triangle fan from the vertices.
    const SurfaceVertexIndex v0{static_cast<int>(surface_uvs.size())};
    for (int i = 0; i < num_vertices; ++i) {
      // The actual vertex positions don't matter.
      surface_vertices_P.emplace_back(Vector3d{0, 0, 0});
      surface_uvs.emplace_back(polygon_uvs[i]);
    }

    // Now build a triangle around v0.
    SurfaceVertexIndex v1{v0 + 1};
    for (int i = 1; i < num_vertices - 1; ++i) {
      const SurfaceVertexIndex v2{v1 + 1};
      surface_faces.emplace_back(v0, v1, v2);
      v1 = v2;
    }
    return BvttCallbackResult::Continue;
  };

  painter_P.bvh->Collide(*canvas_C.bvh, X_PC, callback);

  if (surface_faces.empty()) return {{}, {}};

  // Note: this mesh currently has garbage vertex positions. It's intended only
  //  to provide topology for the uv mesh.
  auto mesh_P = make_unique<SurfaceMesh<double>>(move(surface_faces),
                                                 move(surface_vertices_P));
  DRAKE_DEMAND(static_cast<int>(surface_uvs.size()) == mesh_P->num_vertices());

  return {move(mesh_P), move(surface_uvs)};
}

/* Class that is responsible for creating a volume mesh from the shape indicated
 by a given id. */
class PainterReifier final : public ShapeReifier {
 public:
  /* Returns a volume mesh for the shape indicated by `id`. The `id` must be
   "valid" in the sense:

     1. It must be in the given scene graph.
     2. It must be a supported geometry (currently only cylinder is supported).

   @throws For "invalid" `id`. */
  unique_ptr<PainterMesh> MakePainterMesh(
      GeometryId id, const SceneGraph<double>& scene_graph) {
    const SceneGraphInspector<double>& inspector =
        scene_graph.model_inspector();
    const Shape& painter_shape = inspector.GetShape(id);
    painter_shape.Reify(this, nullptr);
    return move(mesh_);
  }

 private:
  // Currently, we're implementing only a subset of shapes that we have
  // whitelisted.
  using ShapeReifier::ImplementGeometry;

  // Override of unsupported error message in support of MeshPainterSystem.
  void ThrowUnsupportedGeometry(const std::string& shape_name) override {
    throw std::runtime_error(
        fmt::format("MeshPainterSystem only supports Box, Cylinder, Ellipsoid, "
                    "and Sphere as painter objects; {} specified",
                    shape_name));
  }

  void ImplementGeometry(const Cylinder& cylinder, void*) final {
    // TODO(SeanCurtis-TRI): Using the hydroelastic tessellation of the volume
    //  creates tets along the length of the cylinder and on the interior.
    //  That resolution doesn't contribute to this application. For this
    //  application the interior resolution is irrelevant; only resolution that
    //  improves the representation near the surface matters most. Create a
    //  custom volume mesh where the mesh is one layer thick; such that there
    //  are the minimum number of internal vertices.

    // A simple, hard-coded resolution hint that preserves some of the curvature
    // of the cylinder's barrel. A resolution hint of radius gives us a cross
    // section with 8 edges. Cutting that in half doubles it to 16 edges.
    const double resolution_hint = cylinder.radius() / 2;
    auto mesh = make_unique<VolumeMesh<double>>(
        internal::MakeCylinderVolumeMesh<double>(cylinder, resolution_hint));
    auto bvh = make_unique<BoundingVolumeHierarchy<VolumeMesh<double>>>(*mesh);
    mesh_ = make_unique<PainterMesh>(move(mesh), move(bvh));
  }

  void ImplementGeometry(const Box& box, void*) final {
    const double resolution_hint =
        std::max({box.width(), box.depth(), box.height()});
    auto mesh = make_unique<VolumeMesh<double>>(
        internal::MakeBoxVolumeMesh<double>(box, resolution_hint));
    auto bvh = make_unique<BoundingVolumeHierarchy<VolumeMesh<double>>>(*mesh);
    mesh_ = make_unique<PainterMesh>(move(mesh), move(bvh));
  }

  void ImplementGeometry(const Sphere& sphere, void*) final {
    const double resolution_hint = sphere.radius() / 2;
    auto mesh =
        make_unique<VolumeMesh<double>>(internal::MakeSphereVolumeMesh<double>(
            sphere, resolution_hint,
            internal::TessellationStrategy::kSingleInteriorVertex));
    auto bvh = make_unique<BoundingVolumeHierarchy<VolumeMesh<double>>>(*mesh);
    mesh_ = make_unique<PainterMesh>(move(mesh), move(bvh));
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void*) final {
    const double resolution_hint =
        std::max({ellipsoid.a(), ellipsoid.b(), ellipsoid.c()}) / 2;
    auto mesh = make_unique<VolumeMesh<double>>(
        internal::MakeEllipsoidVolumeMesh<double>(
            ellipsoid, resolution_hint,
            internal::TessellationStrategy::kSingleInteriorVertex));
    auto bvh = make_unique<BoundingVolumeHierarchy<VolumeMesh<double>>>(*mesh);
    mesh_ = make_unique<PainterMesh>(move(mesh), move(bvh));
  }

  // We need a pointer because there is no default constructor for a mesh.
  unique_ptr<PainterMesh> mesh_;
};

/* Class that is responsible for creating a surface mesh (with a UV field) that
 can be painted (i.e., a "canvas") from the shape indicated by an id. */
class CanvasReifier final : public ShapeReifier {
 public:
  /* Given SceneGraph and target id, get a mesh representation with UVs. The
   `id` must be "valid" in the sense:

    1. It must be in the given scene graph.
    2. It must refer to a `Mesh` type.
    3. The filename in the mesh must be parseable.
    4. The obj for the mesh must have defined texture coordinates.

   @throws if the id is not "valid".
   */
  unique_ptr<TexturedCanvasMesh> MakeCanvasMesh(
      GeometryId id, const SceneGraph<double>& scene_graph) {
    const SceneGraphInspector<double>& inspector =
        scene_graph.model_inspector();
    const Shape& painter_shape = inspector.GetShape(id);
    painter_shape.Reify(this, nullptr);
    return move(mesh_);
  }

  // TODO(SeanCurtis-TRI): We can create a canvas from arbitrary shapes as long
  //  as we generate UVs for the shapes.

 private:
  using ShapeReifier::ImplementGeometry;

  // Override of unsupported error message in support of MeshPainterSystem.
  void ThrowUnsupportedGeometry(const std::string& shape_name) override {
    throw std::runtime_error(
        fmt::format("MeshPainterSystem only supports Mesh shape types as "
                    "canvas; {} specified",
                    shape_name));
  }

  // Currently, only meshes can be canvases.
  void ImplementGeometry(const geometry::Mesh& mesh_spec, void*) final {
    tinyobj::attrib_t attrib;                    // All vertex data.
    std::vector<tinyobj::shape_t> shapes;        // Triangle data.
    std::vector<tinyobj::material_t> materials;  // Ignored.
    std::string err;

    // Tinyobj doesn't infer the search directory from the directory containing
    // the obj file. We have to provide that directory; of course, this assumes
    // that the material library reference is relative to the obj directory.
    // *Not* providing a valid base directory will cause tiny obj to crash while
    // parsing a file that references a mtllib file.
    const size_t pos = mesh_spec.filename().find_last_of('/');
    const std::string obj_folder = mesh_spec.filename().substr(0, pos + 1);
    const char* mtl_basedir = obj_folder.c_str();

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                mesh_spec.filename().c_str(), mtl_basedir,
                                true /* triangulate */);

    // NOTE: Reading a gibberish file that is *not* an OBJ doesn't necessarily
    // produce "errors". Errors are only produced when it see a line that is
    // _suggestive_ of an OBJ file but then fails. So, it is highly unlikely
    // this exception will be thrown.
    if (!ret || !err.empty()) {
      throw std::runtime_error(fmt::format(
          "Error parsing Wavefront obj file ({}): ", mesh_spec.filename(),
          err));
    }

    // NOTE: This error is far more likely in the case that a bad file is
    // passed. No lines look like OBJ data, so no errors are generated, but also
    // no geometry.
    if (shapes.size() == 0) {
      throw std::runtime_error(
          fmt::format("Parsing file as OBJ file produced no geometry: ({})",
                      mesh_spec.filename()));
    }

    if (attrib.texcoords.size() == 0) {
      throw std::runtime_error(
          fmt::format("OBJ cannot be used as a paint canvas; it has no texture "
                      "coordinates: {}",
                      mesh_spec.filename()));
    }

    // The parsed product needs to be further processed. The SurfaceMesh and
    // SurfaceMeshLinear assumes one-to-one correspondence between vertex
    // positions and field values. However, we_cannot_ assume that
    // each vertex position is uniquely associated with a single texture
    // coordinate in the OBJ file. OBJ allows discontinuous mapping across a
    // vertex; it can associate positions with arbitrary texture coords as it
    // constructs a face. We don't have that luxury in the SurfaceMesh
    // and SurfaceMeshFieldLinear. So, if I have multiple UV values associated
    // with a single position, that position must be duplicated in the final
    // mesh -- each copy associated with a different texture coord in the linear
    // field.
    //
    // To accomplish this end:
    //  1. Every vertex referenced by a face in the parsed OBJ is a "meta"
    //     vertex consisting of a pair of indices: (p, uv), the index to a
    //     position index and a texture coordinate (uv) index.
    //     Imagine one faces refers to meta index (p, uv₀) and another face
    //     refers to index (p, uv₁). Although they have the same _position_ in
    //     generating the surface mesh they use different texture coordinates;
    //     this implies two "meta" vertices in the resultant mesh: the position
    //     copied twice and associated with the two texture coordinates.
    //  2. Given a mapping (p, uv) --> i (a mapping from the meta vertex in the
    //     parsed OBJ data to the unique index in the resultant surface mesh) we
    //     can build the faces in the final SurfaceMesh by mapping the (p, uv)
    //     pair in the OBJ face specification to the final SurfaceMesh vertex
    //     index i.
    //  3. When done, we should have an equal number of vertex positions as
    //     we have uv values. And all indices in the faces should be valid
    //     indices into both vectors of data.

    // The map from (p, uv) --> i.
    map<pair<int, int>, int> obj_vertex_to_new_vertex;
    // Accumulators for vertex positions, texture coordinates, and faces.
    vector<SurfaceVertex<double>> positions;
    vector<Vector2d> uvs;
    vector<SurfaceFace> faces;

    for (const auto& shape : shapes) {
      // The faces are encoded in a series of index values [i0, i1, ..., iN]
      // Each face is a block (e.g., [(i0, i1, i2), (i2, i3, i4), ... ]). We
      // know that the block size must be three. So, we'll march through the
      // faces and maintain an index into the vector of indices so we can always
      // pull out the next index.
      int v_index = 0;
      const auto& shape_mesh = shape.mesh;
      const int num_faces =
          static_cast<int>(shape_mesh.num_face_vertices.size());
      for (int f = 0; f < num_faces; ++f) {
        DRAKE_DEMAND(shape_mesh.num_face_vertices[f] == 3);
        // Captures the [i0, i1, i2] new index values for the face.
        int face_vertices[3] = {-1, -1, -1};
        for (int i = 0; i < 3; ++i) {
          const int position_index = shape_mesh.indices[v_index].vertex_index;
          const int uv_index = shape_mesh.indices[v_index].texcoord_index;
          if (uv_index < 0) {
            throw std::runtime_error(
                fmt::format("OBJ cannot be used as a paint canvas; at least "
                            "one face is missing texture coordinates: {}",
                            mesh_spec.filename()));
          }
          const auto obj_indices = make_pair(position_index, uv_index);
          if (obj_vertex_to_new_vertex.count(obj_indices) == 0) {
            obj_vertex_to_new_vertex[obj_indices] =
                static_cast<int>(positions.size());
            // Guarantee that the positions.size() == uvs.size() by always
            // growing them in lock step.
            positions.emplace_back(
                mesh_spec.scale() *
                Vector3d{attrib.vertices[3 * position_index],
                         attrib.vertices[3 * position_index + 1],
                         attrib.vertices[3 * position_index + 2]});
            uvs.emplace_back(attrib.texcoords[2 * uv_index],
                             attrib.texcoords[2 * uv_index + 1]);
            // Disallow negative texture coordinates.
            if (uvs.back()(0) < 0 || uvs.back()(1) < 0) {
              throw std::runtime_error(fmt::format(
                  "OBJ cannot be used as a paint canvas; at least one texture "
                  "coordinate has negative measures: {}",
                  mesh_spec.filename()));
            }
          }
          face_vertices[i] = obj_vertex_to_new_vertex[obj_indices];
          ++v_index;
        }
        faces.emplace_back(face_vertices);
      }
    }

    // Reports true if a, b ∈ [k, k+1] for some integer k.
    auto in_same_interval = [](double a, double b) {
      const double max_val = std::max(a, b);
      const double min_val = std::min(a, b);
      // They are in the same interval if floor(a) == floor(b) or
      // max_val == k + 1.
      return static_cast<int>(max_val) == static_cast<int>(min_val) ||
             static_cast<int>(max_val) == max_val;
    };

    // Confirm that triangles don't "straddle" image boundaries.
    for (const auto& f : faces) {
      const auto& uv0 = uvs[f.vertex(0)];
      const auto& uv1 = uvs[f.vertex(1)];
      const auto& uv2 = uvs[f.vertex(2)];
      if (!(in_same_interval(uv0(0), uv1(0)) &&
            in_same_interval(uv0(0), uv2(0)) &&
            in_same_interval(uv1(0), uv2(0)) &&
            in_same_interval(uv0(1), uv1(1)) &&
            in_same_interval(uv0(1), uv2(1)) &&
            in_same_interval(uv1(1), uv2(1)))) {
        throw std::runtime_error(fmt::format(
            "OBJ cannot be used as a paint canvas; at least one triangle "
            "spans image boundaries: {}",
            mesh_spec.filename()));
      }
    }

    auto surface_mesh =
        make_unique<SurfaceMesh<double>>(move(faces), move(positions));
    auto bvh = make_unique<BoundingVolumeHierarchy<SurfaceMesh<double>>>(
        *surface_mesh);
    DRAKE_DEMAND(static_cast<int>(uvs.size()) == surface_mesh->num_vertices());
    mesh_ = make_unique<TexturedCanvasMesh>(move(surface_mesh), move(uvs),
                                            move(bvh));
  }

  unique_ptr<TexturedCanvasMesh> mesh_;
};

/* Rasterizes the triangle ABC into the given image. The triangle is given in
 texture coordinates (i.e., (0, 0)x(1, 1) mapping to (0, 0)x(width, height)).

 In this incarnation, the rasterization has the following properties:

   - rasterization color is white.
   - there is no anti-aliasing.
 */
void RasterizeTriangle(const Vector2d& a, const Vector2d& b, const Vector2d& c,
                       ImageRgba8U* image) {
  /* The algorithm is as follows:
     - map uv coordinates to pixels
     - Find the image bounding box that contains the image
     - For each pixel in the bounding region, determine its barycentric
       coordinates.
     - the pixel gets colored if the barycentric coordinate indicates it is
       clearly inside.
   */

  /* Known problems
      1. Computing the barycentric coordinates should be done in an incremental
         manner, not starting from scratch each time.
      2. A triangle that spans (for example) a domain of uvs from [1 - a, 1 + b]
         will render incorrectly.

          ┌┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┐                        ┌┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┐
          ┆                  ┆                        ┆                  ┆
          ┆                  ┆ ╱│                     ┆ ╱│               ┆
          ┆                  ┆╱ │                     ┆╱ │               ┆
          ┆                 ╱┆  │                     ┆  │              ╱┆
          ┆                ╱ ┆  │ should rasterize as ┆  │             ╱ ┆
          ┆                ╲ ┆  │                     ┆  │             ╲ ┆
          ┆                 ╲┆  │                     ┆  │              ╲┆
          ┆                  ┆╲ │                     ┆╲ │               ┆
          ┆                  ┆ ╲│                     ┆ ╲│               ┆
          ┆                  ┆                        ┆                  ┆
          └┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┘                        └┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┘

            It currently rasterizes as:
                            ┌┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┐
                            ┆                  ┆
                            ┆  │---            ┆
                            ┆  │   ---         ┆
                            ┆  │      ----     ┆
                            ┆  │          ---  ┆
                            ┆  │          ---  ┆
                            ┆  │       ---     ┆
                            ┆  │    ---        ┆
                            ┆  │---            ┆
                            ┆                  ┆
                            └┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┘
       In other words, it needs to be split and rasterized as two sections. If
       it overlaps a corner, we need to do more. We're avoiding this problem by
       detecting it parse time and throwing.
   */
  const int w = image->width();
  const int h = image->height();

  /* Maps uv coordinates to "pixel" coordinates. Specifically, such that the
   center of pixel (0, 0) is at <0, 0>. The transformation consists of scaling
   by the image size _and_ then offsetting half a pixel.

   Per-vertex texture coordinates are not necessarily limited to the range
   [0, 1]. Textures get repeated by increasing the domain outside of [0, 1]. In
   fact a single triangle can span multiple repetitions of the periodic UV
   domain. This function does *not* handle that at all well. It simply throws
   out the repetition and places the vertices at their final landing point on
   the image. Furthermore, it requires that all texture coordinate values be
   non-negative. Making this smarter is a prerequisite for removing the
   documented requirements on texture coordinates.
  */
  auto calc_image_coord = [w, h](const Vector2d& uv) {
    const double u = uv[0] - static_cast<int>(uv[0]);
    const double v = uv[1] - static_cast<int>(uv[1]);
    return Vector2d{u * w - 0.5, v * h - 0.5};
  };
  const Vector2d A = calc_image_coord(a);
  const Vector2d B = calc_image_coord(b);
  const Vector2d C = calc_image_coord(c);

  // Finds the bounding box.
  const double min_x_d = std::min(A[0], std::min(B[0], C[0]));
  const double min_y_d = std::min(A[1], std::min(B[1], C[1]));
  const double max_x_d = std::max(A[0], std::max(B[0], C[0]));
  const double max_y_d = std::max(A[1], std::max(B[1], C[1]));

  // Casting int to double truncates -- so we are rounding down (assuming
  // non-negative values).
  const int min_x = std::max(0, static_cast<int>(min_x_d));
  const int min_y = std::max(0, static_cast<int>(min_y_d));
  const int max_x = std::min(w, static_cast<int>(max_x_d) + 1);
  const int max_y = std::min(h, static_cast<int>(max_y_d) + 1);

  DRAKE_DEMAND(min_x <= max_x);
  DRAKE_DEMAND(min_y <= max_y);

  auto calc_bary_i = [](const Vector2d& p, const Vector2d& v0,
                        const Vector2d& v1, const Vector2d& v2) {
    const Vector2d p_v1v0{v0[1] - v1[1], v1[0] - v0[0]};
    const double det_v0v1 = v0[0] * v1[1] - v0[1] * v1[0];
    const double denom = p_v1v0.dot(v2) + det_v0v1;
    // If the denominator is zero, the triangle is degenerate and I can consider
    // that I have *no* contribution from vertex v2 and I should just
    // interpolate between vertices v0 and v1.
    if (denom < 1e-15) return 0.0;
    const double num = p_v1v0.dot(p) + det_v0v1;

    return num / denom;
  };

  // NOTE: Because we're always rasterizing the same color, we don't worry
  // about pixels whose centers lie *exactly* on a triangle edge; we'll simply
  // color them twice.
  for (int x = min_x; x < max_x; ++x) {
    for (int y = min_y; y < max_y; ++y) {
      // NOTE: this is *horribly* inefficient; I'm doing a lot of redundant
      // calculations to compute the barycentric coordinate of each pixel
      // independently. If I did this iteratively it would be *much* faster.
      const double alpha = calc_bary_i({x, y}, B, C, A);
      const double beta = calc_bary_i({x, y}, C, A, B);
      const double gamma = 1 - alpha - beta;
      if (alpha >= 0 && beta >= 0 && gamma >= 0) {
        // TODO(SeanCurtis-TRI): this is type
        //  ImageTraits<PixelType::kRgb8U>::ChannelType*, but I don't want to
        //  handle the name spaces.
        auto* pixel = image->at(x, y);
        *pixel = 0;        // red.
        *(pixel + 1) = 0;  // green.
        *(pixel + 2) = 0;  // blue.
        // NOTE: We are currently relying on alpha to have been pre-configured
        // to be 255 (see the constructor of MeshPainterSystem). If that
        // changes, we may have to set alpha directly.
      }
    }
  }
}

}  // namespace

MeshPainterSystem::MeshPainterSystem(GeometryId canvas_id,
                                     GeometryId painter_id, int width,
                                     int height,
                                     const SceneGraph<double>& scene_graph)
    : LeafSystem<double>(),
      canvas_id_(canvas_id),
      painter_id_(painter_id),
      painter_mesh_(PainterReifier().MakePainterMesh(painter_id, scene_graph)),
      canvas_mesh_(CanvasReifier().MakeCanvasMesh(canvas_id, scene_graph)) {
  geometry_query_input_port_ = &this->DeclareAbstractInputPort(
      "geometry_query", Value<QueryObject<double>>{});

  // TODO(SeanCurtis-TRI): This is *probably* the wrong image type; given this
  //  is a mask, simply an eight-bit luminance value will be enough. However,
  //  that requires SceneGraph to support heterogeneous texture types.

  // Initialize the image with all bytes set to 255 -- a white, fully opaque
  // image. We'll paint opaque black onto the texture.
  ImageRgba8U image(width, height, 255);
  // TODO(SeanCurtis-TRI): Does the image get initialized to black? I'll need to
  //  do so if it hasn't been (or some user-configurable value).
  texture_output_port_ = &this->DeclareAbstractOutputPort(
      "texture", image, &MeshPainterSystem::CalcTextureImage);
}

const InputPort<double>& MeshPainterSystem::geometry_query_input_port() const {
  return *geometry_query_input_port_;
}

const OutputPort<double>& MeshPainterSystem::texture_output_port() const {
  return *texture_output_port_;
}

void MeshPainterSystem::CalcTextureImage(const Context<double>& context,
                                         ImageRgba8U* texture) const {
  const auto& query_object =
      geometry_query_input_port_->Eval<QueryObject<double>>(context);
  // Frames C and P are the frames of the canvas and painter objects,
  // respectively.
  const RigidTransformd& X_WC = query_object.X_WG(canvas_id_);
  const RigidTransformd& X_WP = query_object.X_WG(painter_id_);

  // Perform intersection patch query.
  const PaintedPatch patch =
      CalculatePaintedPatch(*canvas_mesh_, X_WC, *painter_mesh_, X_WP);
  if (patch.mesh == nullptr) return;

  // Rasterize patch onto texture.
  // TODO(SeanCurtis-TRI): This will go away when incrementing is a natural
  //  part of accessing the mutable data.
  const SurfaceMesh<double>& mesh = *patch.mesh;
  const vector<Vector2d>& uvs = patch.uvs;
  for (SurfaceFace face : mesh.faces()) {
    RasterizeTriangle(uvs[face.vertex(0)], uvs[face.vertex(1)],
                      uvs[face.vertex(2)], texture);
  }
}

}  // namespace geometry
}  // namespace drake
