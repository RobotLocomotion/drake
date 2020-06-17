#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/* Definition of a GLSL shader program including a vertex and fragment shader.
 All operations on this shader program require an active OpenGL context. In
 fact, they require the same context which was active when the program was
 "loaded".  */
class ShaderProgram {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ShaderProgram)

  ShaderProgram() = default;

  ~ShaderProgram();

  /* Loads a %ShaderProgram from GLSL code contained in the provided strings.

   @param vertex_shader_source    The valid GLSL source code for a vertex
                                  shader.
   @param fragment_shader_source  The valid GLSL source code for a fragment
                                  shader.
   @throws std::runtime_error if either shader source doesn't compile or the
                              resulting program has errors.
   */
  void LoadFromSources(const std::string& vertex_shader_source,
                       const std::string& fragment_shader_source);

  /* Loads a %ShaderProgram from GLSL code contained in the named files.

   @param vertex_shader_file    The path to a file containing valid GLSL source
                                code for a vertex shader.
   @param fragment_shader_file  The path to a file containing valid GLSL source
                                code for a fragment shader.
   @throws std::runtime_error if there is an error in reading the files, either
                              shader source doesn't compile, or the resulting
                              program has errors.
   */
  void LoadFromFiles(const std::string& vertex_shader_file,
                     const std::string& fragment_shader_file);

  /* Provides the location of the named shader uniform parameter.
   @throws std::runtime_error if the named uniform isn't part of the program. */
  GLint GetUniformLocation(const std::string& uniform_name) const;

  /* Sets the scalar uniform value to the given value.
   @throws std::runtime_error if the named uniform isn't part of the program. */
  void SetUniformValue1f(const std::string& uniform_name, float value) const;

  /* Binds the program for usage.  */
  void Use() const;

  /* Unbinds the program (if this is the current program).  */
  void Unuse() const;

 private:
  friend class ShaderProgramTest;

  GLuint program_id_{0};
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
