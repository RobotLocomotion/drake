#include "drake/geometry/render_gl/internal_geometry_library.h"

#include <fmt/format.h>

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render_gl/internal_opengl_context.h"
#include "drake/geometry/render_gl/internal_opengl_includes.h"
#include "drake/geometry/render_gl/internal_shape_meshes.h"
#include "drake/geometry/shape_to_string.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

using geometry::internal::LoadRenderMeshFromObj;
using geometry::internal::RenderMesh;
using std::vector;

GeometryLibrary::GeometryLibrary(const GeometryLibrary& other) {
  sphere_ = other.sphere_;
  cylinder_ = other.cylinder_;
  half_space_ = other.half_space_;
  box_ = other.box_;
  capsules_ = other.capsules_;
  meshes_ = other.meshes_;

  // Now update the vertex array objects (VAO) for all defined geometries.
  auto update_array = [this](OpenGlGeometry* geometry) {
    if (!geometry->is_defined()) {
      // The geometry data hasn't been defined for this geometry yet.
      // E.g., we're cloning a RenderEngineGl that hasn't instantiated
      // sphere_ yet.
      return;
    }
    // CreateVertexArray will confirm that an appropriate context is bound.
    this->CreateVertexArray(geometry);
  };

  // All stored OpenGlGeometry instances need to rebuild their vertex arrays.
  // Vertex arrays are *not* shared between contexts.
  update_array(&box_);
  update_array(&cylinder_);
  update_array(&half_space_);
  update_array(&sphere_);
  for (auto& capsule : capsules_) {
    update_array(&capsule);
  }
  for (auto& [_, mesh] : meshes_) {
    update_array(&mesh);
  }
}

OpenGlGeometry& GeometryLibrary::GetBox() {
  if (!box_.is_defined()) {
    RenderMesh mesh_data = MakeUnitBox();
    box_ = CreateGlGeometry(mesh_data);
  }

  box_.throw_if_undefined("Built-in box has some invalid objects");

  return box_;
}

OpenGlGeometry GeometryLibrary::GetCapsule(const Capsule& capsule) {
  const int resolution = 50;
  RenderMesh mesh_data = drake::geometry::render_gl::internal::MakeCapsule(
      resolution, capsule.radius(), capsule.length());

  OpenGlGeometry geometry = CreateGlGeometry(mesh_data);

  geometry.throw_if_undefined(fmt::format(
      "Error creating object for capsule {}", ShapeToString(capsule).string()));

  capsules_.push_back(geometry);
  return geometry;
}

OpenGlGeometry& GeometryLibrary::GetCylinder() {
  if (!cylinder_.is_defined()) {
    const int kLongitudeBands = 50;

    // For long skinny cylinders, it would be better to offer some subdivisions
    // along the length. For now, we'll simply save the triangles.
    RenderMesh mesh_data = MakeUnitCylinder(kLongitudeBands, 1);
    cylinder_ = CreateGlGeometry(mesh_data);
  }

  cylinder_.throw_if_undefined("Built-in cylinder has some invalid objects");

  return cylinder_;
}

OpenGlGeometry& GeometryLibrary::GetHalfSpace() {
  if (!half_space_.is_defined()) {
    // This matches the RenderEngineVtk half space size. Keep them matching
    // so that the common "horizon" unit test passes.
    const GLfloat kMeasure = 100.f;
    // TODO(SeanCurtis-TRI): For vertex-lighting (as opposed to fragment
    //  lighting), this will render better with tighter resolution. Consider
    //  making this configurable.
    RenderMesh mesh_data = MakeSquarePatch(kMeasure, 1);
    half_space_ = CreateGlGeometry(mesh_data);
  }

  half_space_.throw_if_undefined(
      "Built-in half space has some invalid objects");

  return half_space_;
}

OpenGlGeometry GeometryLibrary::GetMesh(const std::filesystem::path mesh_path,
                                        const Rgba& default_diffuse) {
  OpenGlGeometry mesh;
  if (meshes_.count(mesh_path.string()) == 0) {
    // TODO(SeanCurtis-TRI): We're ignoring the declared perception properties
    //  for the mesh. We need to pass it in and return a mesh *and* the
    //  resulting material properties.
    RenderMesh mesh_data = LoadRenderMeshFromObj(
        mesh_path.string(), PerceptionProperties(), default_diffuse);
    mesh = CreateGlGeometry(mesh_data);
    meshes_.insert({mesh_path.string(), mesh});
  } else {
    mesh = meshes_[mesh_path.string()];
  }

  mesh.throw_if_undefined(
      fmt::format("Error creating object for mesh {}", mesh_path));

  return mesh;
}

OpenGlGeometry& GeometryLibrary::GetSphere() {
  if (!sphere_.is_defined()) {
    const int kLatitudeBands = 50;
    const int kLongitudeBands = 50;

    RenderMesh mesh_data =
        MakeLongLatUnitSphere(kLongitudeBands, kLatitudeBands);

    sphere_ = CreateGlGeometry(mesh_data);
  }

  sphere_.throw_if_undefined("Built-in sphere has some invalid objects");

  return sphere_;
}

OpenGlGeometry GeometryLibrary::CreateGlGeometry(
    const RenderMesh& mesh_data) const {
  OpenGlGeometry geometry;

  // Create the vertex buffer object (VBO).
  glCreateBuffers(1, &geometry.vertex_buffer);

  // Proof that an OpenGlContext has been bound.
  DRAKE_ASSERT(glIsBuffer, geometry.vertex_buffer);

  // We're representing the vertex data as a concatenation of positions,
  // normals, and texture coordinates (i.e., (VVV...NNN...UU...)). There should
  // be an equal number of vertices, normals, and texture coordinates.
  DRAKE_DEMAND(mesh_data.positions.rows() == mesh_data.normals.rows());
  DRAKE_DEMAND(mesh_data.positions.rows() == mesh_data.uvs.rows());
  const int v_count = mesh_data.positions.rows();
  vector<GLfloat> vertex_data;
  // 3 floats each for position and normal, 2 for texture coordinates.
  const int kFloatsPerPosition = 3;
  const int kFloatsPerNormal = 3;
  const int kFloatsPerUv = 2;
  vertex_data.reserve(v_count *
                      (kFloatsPerPosition + kFloatsPerNormal + kFloatsPerUv));
  // N.B. we are implicitly converting from double to float by inserting them
  // into the vector.
  vertex_data.insert(vertex_data.end(), mesh_data.positions.data(),
                     mesh_data.positions.data() + v_count * kFloatsPerPosition);
  vertex_data.insert(vertex_data.end(), mesh_data.normals.data(),
                     mesh_data.normals.data() + v_count * kFloatsPerNormal);
  vertex_data.insert(vertex_data.end(), mesh_data.uvs.data(),
                     mesh_data.uvs.data() + v_count * kFloatsPerUv);
  glNamedBufferStorage(geometry.vertex_buffer,
                       vertex_data.size() * sizeof(GLfloat), vertex_data.data(),
                       0);

  // Create the index buffer object (IBO).
  using indices_uint_t = decltype(mesh_data.indices)::Scalar;
  static_assert(sizeof(GLuint) == sizeof(indices_uint_t),
                "If this fails, cast from unsigned int to GLuint");
  glCreateBuffers(1, &geometry.index_buffer);
  glNamedBufferStorage(geometry.index_buffer,
                       mesh_data.indices.size() * sizeof(GLuint),
                       mesh_data.indices.data(), 0);

  geometry.index_buffer_size = mesh_data.indices.size();

  geometry.has_tex_coord = mesh_data.has_tex_coord;

  geometry.v_count = v_count;
  CreateVertexArray(&geometry);

  // Note: We won't need to call the corresponding glDeleteVertexArrays or
  // glDeleteBuffers. The meshes we store are "canonical" meshes. Even if a
  // particular GeometryId is removed, it was only referencing its corresponding
  // canonical mesh. We keep all canonical meshes alive for the lifetime of the
  // OpenGL context for convenient reuse.
  return geometry;
}

void GeometryLibrary::CreateVertexArray(OpenGlGeometry* geometry) const {
  DRAKE_ASSERT(OpenGlContext::SomethingBound());
  DRAKE_ASSERT(glIsBuffer(geometry->vertex_buffer));
  DRAKE_ASSERT(glIsBuffer(geometry->index_buffer));
  DRAKE_ASSERT(geometry->v_count > 0);

  glCreateVertexArrays(1, &geometry->vertex_array);

  // 3 floats each for position and normal, 2 for texture coordinates.
  const int kFloatsPerPosition = 3;
  const int kFloatsPerNormal = 3;
  const int kFloatsPerUv = 2;

  std::size_t vbo_offset = 0;

  const int position_attrib = 0;
  glVertexArrayVertexBuffer(geometry->vertex_array, position_attrib,
                            geometry->vertex_buffer, vbo_offset,
                            kFloatsPerPosition * sizeof(GLfloat));
  glVertexArrayAttribFormat(geometry->vertex_array, position_attrib,
                            kFloatsPerPosition, GL_FLOAT, GL_FALSE, 0);
  glEnableVertexArrayAttrib(geometry->vertex_array, position_attrib);
  vbo_offset += geometry->v_count * kFloatsPerPosition * sizeof(GLfloat);

  const int normal_attrib = 1;
  glVertexArrayVertexBuffer(geometry->vertex_array, normal_attrib,
                            geometry->vertex_buffer, vbo_offset,
                            kFloatsPerNormal * sizeof(GLfloat));
  glVertexArrayAttribFormat(geometry->vertex_array, normal_attrib,
                            kFloatsPerNormal, GL_FLOAT, GL_FALSE, 0);
  glEnableVertexArrayAttrib(geometry->vertex_array, normal_attrib);
  vbo_offset += geometry->v_count * kFloatsPerNormal * sizeof(GLfloat);

  const int uv_attrib = 2;
  glVertexArrayVertexBuffer(geometry->vertex_array, uv_attrib,
                            geometry->vertex_buffer, vbo_offset,
                            kFloatsPerUv * sizeof(GLfloat));
  glVertexArrayAttribFormat(geometry->vertex_array, uv_attrib, kFloatsPerUv,
                            GL_FLOAT, GL_FALSE, 0);
  glEnableVertexArrayAttrib(geometry->vertex_array, uv_attrib);
  vbo_offset += geometry->v_count * kFloatsPerUv * sizeof(GLfloat);

  const float float_count =
      geometry->v_count *
      (kFloatsPerPosition + kFloatsPerNormal + kFloatsPerUv);
  DRAKE_DEMAND(vbo_offset == float_count * sizeof(GLfloat));

  // Bind IBO with the VAO.
  glVertexArrayElementBuffer(geometry->vertex_array, geometry->index_buffer);
}

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
