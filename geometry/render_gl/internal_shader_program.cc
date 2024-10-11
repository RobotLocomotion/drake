#include "drake/geometry/render_gl/internal_shader_program.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;

namespace {

GLuint CompileShader(GLuint shader_type, const std::string& shader_code) {
  GLuint shader_gl_id = glCreateShader(shader_type);
  char const* source_ptr = shader_code.c_str();
  glShaderSource(shader_gl_id, 1, &source_ptr, NULL);
  glCompileShader(shader_gl_id);

  // Check compilation result.
  GLint is_compiled{0};
  glGetShaderiv(shader_gl_id, GL_COMPILE_STATUS, &is_compiled);
  if (is_compiled == GL_FALSE) {
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

GLuint LinkProgram(GLuint vertex_id, GLuint fragment_id) {
  // Link.
  GLuint program_id = glCreateProgram();
  glAttachShader(program_id, vertex_id);
  glAttachShader(program_id, fragment_id);
  glLinkProgram(program_id);

  // Partial clean up; we'll detach the shaders from the program, but we won't
  // delete them so we can reuse them in cloning.
  glDetachShader(program_id, vertex_id);
  glDetachShader(program_id, fragment_id);

  // Check.
  GLint result{0};
  glGetProgramiv(program_id, GL_LINK_STATUS, &result);
  if (!result) {
    const std::string error_prefix = "Error linking shaders: ";
    std::string info("No further information available");
    int info_log_length;
    glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
      std::vector<char> error_message(info_log_length + 1);
      glGetProgramInfoLog(program_id, info_log_length, NULL, &error_message[0]);
      info = &error_message[0];
    }
    throw std::runtime_error(error_prefix + info);
  }
  return program_id;
}

}  // namespace

void ShaderProgram::LoadFromSources(const std::string& vertex_shader_source,
                                    const std::string& fragment_shader_source) {
  vertex_id_ = CompileShader(GL_VERTEX_SHADER, vertex_shader_source);
  fragment_id_ = CompileShader(GL_FRAGMENT_SHADER, fragment_shader_source);

  gl_id_ = LinkProgram(vertex_id_, fragment_id_);

  ConfigureUniforms();
}

namespace {
std::string LoadFile(const std::string& filename) {
  std::optional<std::string> content = ReadFile(filename);
  if (!content)
    throw std::runtime_error("Error opening shader file: " + filename);
  return std::move(*content);
}
}  // namespace

ShaderProgram::~ShaderProgram() = default;

void ShaderProgram::LoadFromFiles(const std::string& vertex_shader_file,
                                  const std::string& fragment_shader_file) {
  LoadFromSources(LoadFile(vertex_shader_file), LoadFile(fragment_shader_file));
}

void ShaderProgram::SetProjectionMatrix(const Eigen::Matrix4f& T_DC) const {
  glUniformMatrix4fv(projection_matrix_loc_, 1, GL_FALSE, T_DC.data());
}

void ShaderProgram::SetModelViewMatrix(const Eigen::Matrix4f& X_CW,
                                       const Eigen::Matrix4f& T_WM,
                                       const Eigen::Matrix3f& N_WM) const {
  const Eigen::Matrix4f T_CM = X_CW * T_WM;
  // Our camera frame C wrt the OpenGL's camera frame Cgl.
  // clang-format off
  static const Eigen::Matrix4f kT_CglC =
      (Eigen::Matrix4f() << 1,  0,  0, 0,
                            0, -1,  0, 0,
                            0,  0, -1, 0,
                            0,  0,  0, 1)
          .finished();
  // clang-format on
  const Eigen::Matrix4f T_CglM = kT_CglC * T_CM;
  glUniformMatrix4fv(model_view_loc_, 1, GL_FALSE, T_CglM.data());
  DoSetModelViewMatrix(X_CW, T_WM, N_WM);
}

GLint ShaderProgram::GetUniformLocation(const std::string& uniform_name) const {
  GLint id = glGetUniformLocation(gl_id_, uniform_name.c_str());
  if (id < 0) {
    throw std::runtime_error(
        fmt::format("Cannot get shader uniform '{}'", uniform_name));
  }
  return id;
}

void ShaderProgram::Use() const {
  glUseProgram(gl_id_);
}

void ShaderProgram::Unuse() const {
  GLint curr_program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &curr_program);
  if (curr_program == static_cast<GLint>(gl_id_)) {
    glUseProgram(0);
  }
}

void ShaderProgram::Relink() {
  DRAKE_ASSERT(glIsShader(vertex_id_));
  gl_id_ = LinkProgram(vertex_id_, fragment_id_);
  ConfigureUniforms();
}

void ShaderProgram::Free() {
  DRAKE_ASSERT(glIsProgram(gl_id_));
  glDeleteProgram(gl_id_);
}

void ShaderProgram::ConfigureUniforms() {
  projection_matrix_loc_ = GetUniformLocation("T_DC");
  model_view_loc_ = GetUniformLocation("T_CM");
  DoConfigureUniforms();
}

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
