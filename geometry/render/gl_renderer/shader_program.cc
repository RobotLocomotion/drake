#include "drake/geometry/render/gl_renderer/shader_program.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace render {
namespace internal {

using Eigen::Vector3d;

namespace {

GLuint CompileShader(GLuint shader_type, const std::string& shader_code) {
  GLuint shader_gl_id = glCreateShader(shader_type);
  char const* source_ptr = shader_code.c_str();
  glShaderSource(shader_gl_id, 1, &source_ptr, NULL);
  glCompileShader(shader_gl_id);

  // Check compilation result.
  GLint result{0};
  glGetShaderiv(shader_gl_id, GL_COMPILE_STATUS, &result);
  if (!result) {
    const std::string error_prefix =
        fmt::format("Error compiling {} shader: ",
                    shader_type == GL_VERTEX_SHADER ? "vertex" : "fragment");
    std::string info("No further information available");
    int info_log_length;
    glGetShaderiv(shader_gl_id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
      std::vector<char> error_message(info_log_length + 1);
      glGetShaderInfoLog(shader_gl_id, info_log_length, NULL,
                         &error_message[0]);
      info = &error_message[0];
    }
    throw std::runtime_error(error_prefix + info);
  }

  return shader_gl_id;
}

}  // namespace

void ShaderProgram::LoadFromSources(const std::string& vertex_shader_source,
                                    const std::string& fragment_shader_source) {
  // Compile.
  GLuint vertex_shader_id =
      CompileShader(GL_VERTEX_SHADER, vertex_shader_source);
  GLuint fragment_shader_id =
      CompileShader(GL_FRAGMENT_SHADER, fragment_shader_source);

  // Link.
  gl_id_ = glCreateProgram();
  glAttachShader(gl_id_, vertex_shader_id);
  glAttachShader(gl_id_, fragment_shader_id);
  glLinkProgram(gl_id_);

  // Clean up.
  glDetachShader(gl_id_, vertex_shader_id);
  glDetachShader(gl_id_, fragment_shader_id);
  glDeleteShader(vertex_shader_id);
  glDeleteShader(fragment_shader_id);

  // Check.
  GLint result{0};
  glGetProgramiv(gl_id_, GL_LINK_STATUS, &result);
  if (!result) {
    const std::string error_prefix = "Error linking shaders: ";
    std::string info("No further information available");
    int info_log_length;
    glGetProgramiv(gl_id_, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
      std::vector<char> error_message(info_log_length + 1);
      glGetProgramInfoLog(gl_id_, info_log_length, NULL,
                          &error_message[0]);
      info = &error_message[0];
    }
    throw std::runtime_error(error_prefix + info);
  }

  projection_matrix_loc_ = GetUniformLocation("T_DC");
  model_view_loc_ = GetUniformLocation("T_CM");
}

namespace {
std::string LoadFile(const std::string& filename) {
  std::ifstream file(filename.c_str(), std::ios::in);
  if (!file.is_open())
    throw std::runtime_error("Error opening shader file: " + filename);
  std::stringstream content;
  content << file.rdbuf();
  file.close();
  return content.str();
}
}  // namespace

void ShaderProgram::LoadFromFiles(const std::string& vertex_shader_file,
                                  const std::string& fragment_shader_file) {
  LoadFromSources(LoadFile(vertex_shader_file), LoadFile(fragment_shader_file));
}

void ShaderProgram::SetProjectionMatrix(const Eigen::Matrix4f& T_DC) const {
  glUniformMatrix4fv(projection_matrix_loc_, 1, GL_FALSE, T_DC.data());
}

void ShaderProgram::SetModelViewMatrix(const Eigen::Matrix4f& X_CM,
                                       const Vector3d& scale) const {
  const Eigen::DiagonalMatrix<float, 4, 4> scale_mat(
      Vector4<float>(scale(0), scale(1), scale(2), 1.0));
  // Our camera frame C wrt the OpenGL's camera frame Cgl.
  // clang-format off
  static const Eigen::Matrix4f kX_CglC =
      (Eigen::Matrix4f() << 1,  0,  0, 0,
                            0, -1,  0, 0,
                            0,  0, -1, 0,
                            0,  0,  0, 1)
          .finished();
  // clang-format on
  const Eigen::Matrix4f X_CglM = kX_CglC * X_CM;
  Eigen::Matrix4f T_CglM = X_CglM * scale_mat;
  glUniformMatrix4fv(model_view_loc_, 1, GL_FALSE, T_CglM.data());
  DoModelViewMatrix(X_CglM, scale);
}

GLint ShaderProgram::GetUniformLocation(const std::string& uniform_name) const {
  GLint id = glGetUniformLocation(gl_id_, uniform_name.c_str());
  if (id < 0) {
    throw std::runtime_error(
        fmt::format("Cannot get shader uniform '{}'", uniform_name));
  }
  return id;
}

void ShaderProgram::Use() const { glUseProgram(gl_id_); }

void ShaderProgram::Unuse() const {
  GLint curr_program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &curr_program);
  if (curr_program == static_cast<GLint>(gl_id_)) {
    glUseProgram(0);
  }
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
