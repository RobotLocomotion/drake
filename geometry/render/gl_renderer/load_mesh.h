#pragma once

#include <string>
#include <utility>

#include <Eigen/Dense>

#include "perception/gl_renderer/opengl_includes.h"

namespace anzu {
namespace gl_renderer {

using VertexBuffer =
    Eigen::Matrix<GLfloat, Eigen::Dynamic, 3, Eigen::RowMajor>;
using IndexBuffer = Eigen::Matrix<GLuint, Eigen::Dynamic, 3, Eigen::RowMajor>;

/// Loads a mesh's vertices and indices (faces). Does not load textures.
std::pair<VertexBuffer, IndexBuffer> LoadMeshFromObj(
    const std::string& filename);

}  // namespace gl_renderer
}  // namespace anzu
