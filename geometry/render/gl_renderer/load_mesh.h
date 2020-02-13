#pragma once

#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
namespace gl {

using VertexBuffer = Eigen::Matrix<GLfloat, Eigen::Dynamic, 3, Eigen::RowMajor>;
using IndexBuffer = Eigen::Matrix<GLuint, Eigen::Dynamic, 3, Eigen::RowMajor>;

/** Loads a mesh's vertices and indices (faces). Does not load textures.
 Note that while this functionality seems similar to ReadObjToSurfaceMesh,
 RenderEngineGl cannot use SurfaceMesh. Rendering requires normals and texture
 coordinates; SurfaceMesh was not designed with those quantities in mind.
 */
std::pair<VertexBuffer, IndexBuffer> LoadMeshFromObj(
    const std::string& filename);

}  // namespace gl
}  // namespace render
}  // namespace geometry
}  // namespace drake
