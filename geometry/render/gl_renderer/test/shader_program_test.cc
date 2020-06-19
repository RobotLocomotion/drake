#include "drake/geometry/render/gl_renderer/dev/shader_program.h"

#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/gl_renderer/opengl_context.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

const char* simple_vertex_source = R"_(
  #version 330
  void main() {
    gl_Position = vec4(0.0, 0.0, 0.0, 1.0);
  }
)_";

const char* simple_fragment_source = R"_(
  #version 330
  uniform float test_uniform;
  void main() {
    gl_FragColor = vec4(0.18, 0.54, 0.34, test_uniform);
  }
)_";

}  // namespace

using std::string;

// Test class is *not* in the anonymous namespace so it can exploit the
// declared friend status in ShaderProgram.
class ShaderProgramTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // ShaderProgram requires an active OpenGL context to perform.
    opengl_context_ = std::make_shared<OpenGlContext>();
  }

  // Reports the program id of the given program.
  static GLuint program_id(const ShaderProgram& program) {
    return program.program_id_;
  }

 private:
  std::shared_ptr<OpenGlContext> opengl_context_;
};

// Test the shader program loading. This tests both sources stored in strings as
// well as reading from disk. We've created an immensely simple shader and just
// want to make sure it loads/compiles without difficulty.
TEST_F(ShaderProgramTest, LoadShaders) {
  {
    // Case: Load from in-memory vertex and fragment shaders.
    ShaderProgram program;
    EXPECT_EQ(program_id(program), 0);
    EXPECT_NO_THROW(
        program.LoadFromSources(simple_vertex_source, simple_fragment_source));
    EXPECT_NE(program_id(program), 0);
  }

  {
    // Case: Load the same shaders from disk.
    auto write_shader = [](const string& file_name, const char* shader) {
      std::ofstream file(file_name);
      if (!file) throw std::logic_error("Can't write file");
      file << shader;
    };

    const string temp_dir = temp_directory();
    const string vertex_shader(temp_dir + "/vertex.glsl");
    const string fragment_shader(temp_dir + "/fragment.glsl");

    write_shader(vertex_shader, simple_vertex_source);
    write_shader(fragment_shader, simple_fragment_source);

    ShaderProgram program;
    EXPECT_NO_THROW(program.LoadFromFiles(vertex_shader, fragment_shader));
    EXPECT_NE(program_id(program), 0);
  }
}

// Confirms the error conditions for loading vertex and fragment shaders. These
// exercise ShaderProgram::LoadFromSources, relying on the fact that
// ShaderProgram::LoadFromFiles ultimately exercises those.
TEST_F(ShaderProgramTest, LoadShadersError) {
  // We'll create a "bad" shader by simply passing garbage. We're ignoring the
  // details of the compilation error reported by the OpenGL driver. The
  // error regular expression uses "[^]+" instead of ".+" because the error
  // may include line breaks and ".+" does *not* include line breaks.
    ShaderProgram program;
  {
    // Case: Bad vertex shader.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromSources("This is garbage", simple_fragment_source),
        std::runtime_error,
        "Error compiling vertex shader[^]+");
  }
  {
    // Case: Bad fragment shader.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromSources(simple_vertex_source, "This is garbage"),
        std::runtime_error,
        "Error compiling fragment shader[^]+");
  }
  {
    // Case: Both shaders are bad. This stops at reporting the vertex error.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromSources("This is garbage", "also garbage"),
        std::runtime_error,
        "Error compiling vertex shader[^]+");
  }
  {
    // Case: linker error. To trigger the linker error, we omit the main
    // function from the fragment shader as documented here:
    // https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glLinkProgram.xhtml
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromSources(simple_vertex_source, R"_(
            #version 100
            void foo() {
              gl_FragColor = vec4(0.1, 0.2, 0.3, 1.0);
            })_"),
        std::runtime_error, "Error linking shaders[^]+");
  }

  {
    // Case: file referenced is not available.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromFiles("invalid.vert", "invalid.frag"),
        std::runtime_error, "Error opening shader file: .+");
  }
}

TEST_F(ShaderProgramTest, UniformAccess) {
  ShaderProgram program;
  program.LoadFromSources(simple_vertex_source, simple_fragment_source);
  EXPECT_NO_THROW(program.GetUniformLocation("test_uniform"));
  DRAKE_EXPECT_THROWS_MESSAGE(program.GetUniformLocation("invalid"),
                              std::runtime_error,
                              "Cannot get shader uniform invalid");
  EXPECT_NO_THROW(program.SetUniformValue1f("test_uniform", 1.5f));
  DRAKE_EXPECT_THROWS_MESSAGE(program.SetUniformValue1f("invalid", 1.75f),
                              std::runtime_error,
                              "Cannot get shader uniform invalid");
}

TEST_F(ShaderProgramTest, Binding) {
  ShaderProgram program1;
  program1.LoadFromSources(simple_vertex_source, simple_fragment_source);
  ShaderProgram program2;
  program2.LoadFromSources(simple_vertex_source, simple_fragment_source);

  // Report the id of the currently bound program.
  auto current_program = []() {
    GLint curr_program;
    glGetIntegerv(GL_CURRENT_PROGRAM, &curr_program);
    return static_cast<GLuint>(curr_program);
  };

  // We start with nothing bound; creating a program isn't the same as binding
  // it.
  ASSERT_EQ(current_program(), 0);

  // Bind one of the programs.
  ASSERT_NO_THROW(program1.Use());
  ASSERT_EQ(current_program(), program_id(program1));

  // Binding another causes the current program to switch.
  ASSERT_NO_THROW(program2.Use());
  ASSERT_EQ(current_program(), program_id(program2));

  // Attempting to unbind with the wrong program has no effect.
  ASSERT_NO_THROW(program1.Unuse());
  ASSERT_EQ(current_program(), program_id(program2));

  // Unbinding the current program clears it.
  ASSERT_NO_THROW(program2.Unuse());
  ASSERT_EQ(current_program(), 0);
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
