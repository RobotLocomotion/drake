#include "drake/geometry/render/gl_renderer/dev/shader_program.h"

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

namespace {

GLuint CompileShader(GLuint shader_type, const std::string& shader_code) {
  GLuint shader_id = glCreateShader(shader_type);
  char const* source_ptr = shader_code.c_str();
  glShaderSource(shader_id, 1, &source_ptr, NULL);
  glCompileShader(shader_id);

  // Check compilation result.
  GLint result{0};
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  if (!result) {
    const std::string error_prefix =
        fmt::format("Error compiling {} shader: ",
                    shader_type == GL_VERTEX_SHADER ? "vertex" : "fragment");
    std::string info("No further information available");
    int info_log_length;
    glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
      std::vector<char> error_message(info_log_length + 1);
      glGetShaderInfoLog(shader_id, info_log_length, NULL, &error_message[0]);
      info = &error_message[0];
    }
    throw std::runtime_error(error_prefix + info);
  }

  return shader_id;
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
  program_id_ = glCreateProgram();
  glAttachShader(program_id_, vertex_shader_id);
  glAttachShader(program_id_, fragment_shader_id);
  glLinkProgram(program_id_);

  // Clean up.
  glDetachShader(program_id_, vertex_shader_id);
  glDetachShader(program_id_, fragment_shader_id);
  glDeleteShader(vertex_shader_id);
  glDeleteShader(fragment_shader_id);

  // Check.
  GLint result{0};
  glGetProgramiv(program_id_, GL_LINK_STATUS, &result);
  if (!result) {
    const std::string error_prefix = "Error linking shaders: ";
    std::string info("No further information available");
    int info_log_length;
    glGetProgramiv(program_id_, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
      std::vector<char> error_message(info_log_length + 1);
      glGetProgramInfoLog(program_id_, info_log_length, NULL,
                          &error_message[0]);
      info = &error_message[0];
    }
    throw std::runtime_error(error_prefix + info);
  }
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

GLint ShaderProgram::GetUniformLocation(const std::string& uniform_name) const {
  GLint id = glGetUniformLocation(program_id_, uniform_name.c_str());
  if (id < 0) {
    throw std::runtime_error("Cannot get shader uniform " + uniform_name);
  }
  return id;
}

void ShaderProgram::SetUniformValue1f(const std::string& uniform_name,
                                      float value) const {
  glUniform1f(GetUniformLocation(uniform_name), value);
}

void ShaderProgram::Use() const { glUseProgram(program_id_); }

void ShaderProgram::Unuse() const {
  GLint curr_program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &curr_program);
  if (curr_program == static_cast<GLint>(program_id_)) {
    glUseProgram(0);
  }
}

ShaderProgram::~ShaderProgram() {
  Unuse();
  glDeleteProgram(program_id_);
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
