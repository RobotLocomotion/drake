#include "drake/geometry/render/gl_renderer/shader_program.h"

#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/pointer_cast.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/gl_renderer/opengl_context.h"
#include "drake/geometry/render/gl_renderer/shader_program_data.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
class ShaderProgramTest;
namespace {

/* A simple shader implementation that exercises all of the virtual API and
 reports if it has been called.  */
class TestShader final : public ShaderProgram {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestShader)

  struct Data {
    int i_value{-1};
    double d_value{-1.0};
  };

  explicit TestShader(int magic_number = 0)
      : ShaderProgram(), magic_number_(magic_number) {}

  // Collection of flags which report if certain virtual methods have been
  // invoked.
  bool DoModelViewMatrixCalled() const { return do_mv_matrix_called_; }

  bool CalledSetInstanceParameters() const {
    return set_instance_params_called_;
  }

  bool CalledSetDepthCameraParameters() const {
    return set_depth_camera_called_;
  }

  // Overrides of virtual methods.
  void SetInstanceParameters(const ShaderProgramData&) const override {
    set_instance_params_called_ = true;
  }

  void SetDepthCameraParameters(
      const DepthRenderCamera& /* camera */) const override {
    set_depth_camera_called_ = true;
  }

 private:
  friend class drake::geometry::render::internal::ShaderProgramTest;

  std::unique_ptr<ShaderProgram> DoClone() const final {
    return std::make_unique<TestShader>(*this);
  }

  // We extract ("test", "i_value") and ("test", "d_value") properties and
  // stash them in
  std::optional<ShaderProgramData> DoCreateProgramData(
      const PerceptionProperties& properties) const override {
    Data data;
    data.i_value = properties.GetProperty<int>("test", "i_value");
    data.d_value = properties.GetProperty<double>("test", "d_value");
    return ShaderProgramData{shader_id(), AbstractValue::Make(data)};
  }

  void DoModelViewMatrix(const Eigen::Matrix4f&,
                         const Eigen::Vector3d&) const override {
    do_mv_matrix_called_ = true;
  }

  mutable bool do_mv_matrix_called_{false};
  mutable bool set_instance_params_called_{false};
  mutable bool set_depth_camera_called_{false};

  int magic_number_{};
};

constexpr char kVertexSource[] = R"""(
  #version 330
  uniform mat4 T_CM;
  uniform mat4 T_DC;
  out vec4 p_CV;
  void main() {
    gl_Position = T_DC * vec4(0.0, 0.0, 0.0, 1.0);
    p_CV = T_CM * vec4(0.0, 0.0, 0.0, 1.0);
  }
)""";

constexpr char kFragmentSource[] = R"""(
  #version 330
  uniform float test_uniform;
  in vec4 p_CV;
  void main() {
    gl_FragColor = vec4(0.18, 0.54, 0.34, test_uniform) + p_CV;
  }
)""";

}  // namespace

using Eigen::Matrix4f;
using Eigen::Vector3d;
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
  static GLuint gl_id(const ShaderProgram& program) { return program.gl_id_; }

  static GLint proj_mat_loc(const ShaderProgram& program) {
    return program.projection_matrix_loc_;
  }

  static GLint model_view_loc(const ShaderProgram& program) {
    return program.model_view_loc_;
  }

  static ::testing::AssertionResult Equals(const TestShader& p1,
                                           const TestShader& p2) {
    if (p1.id_ != p2.id_ || p1.gl_id_ != p2.gl_id_ ||
        p1.projection_matrix_loc_ != p2.projection_matrix_loc_ ||
        p1.model_view_loc_ != p2.model_view_loc_ ||
        p1.magic_number_ != p2.magic_number_) {
      return ::testing::AssertionFailure()
             << "Shader programs didn't match"
             << "\n  p1:"
             << "\n    shader id: " << p1.id_
             << "\n    OpenGl id: " << p1.gl_id_
             << "\n    projection matrix location: "
             << p1.projection_matrix_loc_
             << "\n    model view matrix location: " << p1.model_view_loc_
             << "\n    magic number: " << p1.magic_number_ << "\n  p2:"
             << "\n    shader id: " << p2.id_
             << "\n    OpenGl id: " << p2.gl_id_
             << "\n    projection matrix location: "
             << p2.projection_matrix_loc_
             << "\n    model view matrix location: " << p2.model_view_loc_
             << "\n    magic number: " << p1.magic_number_;
    }
    return ::testing::AssertionSuccess();
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
    TestShader program;
    EXPECT_EQ(gl_id(program), 0);
    EXPECT_NO_THROW(program.LoadFromSources(kVertexSource, kFragmentSource));
    EXPECT_NE(gl_id(program), 0);
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

    write_shader(vertex_shader, kVertexSource);
    write_shader(fragment_shader, kFragmentSource);

    TestShader program;
    EXPECT_NO_THROW(program.LoadFromFiles(vertex_shader, fragment_shader));
    EXPECT_NE(gl_id(program), 0);
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
  TestShader program;
  {
    // Case: Bad vertex shader.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromSources("This is garbage", kFragmentSource),
        std::runtime_error, "Error compiling vertex shader[^]+");
  }

  {
    // Case: Bad fragment shader.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromSources(kVertexSource, "This is garbage"),
        std::runtime_error, "Error compiling fragment shader[^]+");
  }

  {
    // Case: Both shaders are bad. This stops at reporting the vertex error.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromSources("This is garbage", "also garbage"),
        std::runtime_error, "Error compiling vertex shader[^]+");
  }

  {
    // Case: linker error. To trigger the linker error, we omit the main
    // function from the fragment shader as documented here:
    // https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glLinkProgram.xhtml
    DRAKE_EXPECT_THROWS_MESSAGE(program.LoadFromSources(kVertexSource, R"""(
            #version 100
            void foo() {
              gl_FragColor = vec4(0.1, 0.2, 0.3, 1.0);
            })"""),
                                std::runtime_error,
                                "Error linking shaders[^]+");
  }

  {
    // Case: file referenced is not available.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromFiles("invalid.vert", "invalid.frag"),
        std::runtime_error, "Error opening shader file: .+");
  }

  {
    // Case: Missing projection matrix uniform. In this case, one is *listed*
    // in the shader, but because it isn't used, it gets compiled out.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromSources(R"""(
  #version 330
  uniform mat4 T_CM;
  uniform mat4 T_DC;
  out vec4 p_CV;
  void main() {
    gl_Position = vec4(0.0, 0.0, 0.0, 1.0);
    p_CV = T_CM * vec4(0.0, 0.0, 0.0, 1.0);
  }
)""",
                                kFragmentSource),
        std::runtime_error, "Cannot get shader uniform 'T_DC'");
  }

  {
    // Case: Missing modelview matrix uniform. In this case, one is *listed*
    // in the shader, but because it isn't used, it gets compiled out.
    DRAKE_EXPECT_THROWS_MESSAGE(
        program.LoadFromSources(R"""(
  #version 330
  uniform mat4 T_CM;
  uniform mat4 T_DC;
  out vec4 p_CV;
  void main() {
    gl_Position = T_DC * vec4(0.0, 0.0, 0.0, 1.0);
    p_CV = vec4(0.0, 0.0, 0.0, 1.0);
  }
)""",
                                kFragmentSource),
        std::runtime_error, "Cannot get shader uniform 'T_CM'");
  }
}

TEST_F(ShaderProgramTest, UniformAccess) {
  TestShader program;
  program.LoadFromSources(kVertexSource, kFragmentSource);
  EXPECT_NO_THROW(program.GetUniformLocation("test_uniform"));
  DRAKE_EXPECT_THROWS_MESSAGE(program.GetUniformLocation("invalid"),
                              std::runtime_error,
                              "Cannot get shader uniform 'invalid'");
}

TEST_F(ShaderProgramTest, Binding) {
  TestShader program1;
  program1.LoadFromSources(kVertexSource, kFragmentSource);
  TestShader program2;
  program2.LoadFromSources(kVertexSource, kFragmentSource);

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
  ASSERT_EQ(current_program(), gl_id(program1));

  // Binding another causes the current program to switch.
  ASSERT_NO_THROW(program2.Use());
  ASSERT_EQ(current_program(), gl_id(program2));

  // Attempting to unbind with the wrong program has no effect.
  ASSERT_NO_THROW(program1.Unuse());
  ASSERT_EQ(current_program(), gl_id(program2));

  // Unbinding the current program clears it.
  ASSERT_NO_THROW(program2.Unuse());
  ASSERT_EQ(current_program(), 0);
}

// Confirms the virtual nature of CreateProgramData() and its respect for the
// override.
TEST_F(ShaderProgramTest, CreateProgramData) {
  TestShader shader;
  ShaderProgram* shader_ptr = &shader;
  shader_ptr->LoadFromSources(kVertexSource, kFragmentSource);

  PerceptionProperties props;
  const int i_value{17};
  const double d_value{18.5};
  props.AddProperty("test", "i_value", i_value);
  props.AddProperty("test", "d_value", d_value);

  std::optional<ShaderProgramData> data = shader_ptr->CreateProgramData(props);
  ASSERT_NE(data, std::nullopt);
  const auto& data_value = data->value().get_value<TestShader::Data>();
  EXPECT_EQ(data_value.i_value, i_value);
  EXPECT_EQ(data_value.d_value, d_value);
  EXPECT_EQ(data->shader_id(), shader.shader_id());
}

// Confirms that the SetInstanceParameters virtual API is respected in derived
// classes.
TEST_F(ShaderProgramTest, SetInstanceParameters) {
  TestShader shader;
  const ShaderProgram* shader_ptr = &shader;

  ShaderProgramData data{ShaderId::get_new_id(), nullptr};
  ASSERT_FALSE(shader.CalledSetInstanceParameters());
  shader_ptr->SetInstanceParameters(data);
  ASSERT_TRUE(shader.CalledSetInstanceParameters());
}

// Confirms the virtual nature of SetDepthCameraParameters().
TEST_F(ShaderProgramTest, SetDepthCameraParameters) {
  TestShader shader;
  const ShaderProgram* shader_ptr = &shader;

  DepthRenderCamera camera{
      RenderCameraCore{"n/a", {10, 10, M_PI}, {0.1, 10.1}, {}},
      DepthRange{0.1, 10}};

  ASSERT_FALSE(shader.CalledSetDepthCameraParameters());
  shader_ptr->SetDepthCameraParameters(camera);
  ASSERT_TRUE(shader.CalledSetDepthCameraParameters());
}

// Confirms that given matrix propagates into the OpenGl state.
TEST_F(ShaderProgramTest, SetProjectionMatrix) {
  TestShader shader;
  shader.LoadFromSources(kVertexSource, kFragmentSource);

  // Note: this is a weird, invalid projection matrix that we shouldn't expect
  // should ever happen by accident.
  const Matrix4f proj_mat = -Matrix4f::Ones();
  shader.Use();
  shader.SetProjectionMatrix(proj_mat);
  shader.Unuse();
  float proj_mat_data[16];
  glGetUniformfv(gl_id(shader), proj_mat_loc(shader), &proj_mat_data[0]);
  const Matrix4f gl_proj_mat(proj_mat_data);
  EXPECT_TRUE(CompareMatrices(proj_mat, gl_proj_mat));
}

// Confirms that given matrix propagates into the OpenGl state and that the
// virtual API gets exercised.
TEST_F(ShaderProgramTest, SetModelViewMatrix) {
  TestShader shader;
  shader.LoadFromSources(kVertexSource, kFragmentSource);
  const ShaderProgram* shader_ptr = &shader;

  // Note: this is a weird, invalid projection matrix that we shouldn't expect
  // should ever happen by accident.
  Matrix4f X_CM = Matrix4f::Identity();
  X_CM.col(3) << 1, 2, 3, 1;  // Simple translation.
  const Vector3d scale(0.5, 2, 4);
  ASSERT_FALSE(shader.DoModelViewMatrixCalled());
  shader_ptr->Use();
  shader_ptr->SetModelViewMatrix(X_CM, scale);
  shader_ptr->Unuse();
  ASSERT_TRUE(shader.DoModelViewMatrixCalled());

  float mv_mat_data[16];
  glGetUniformfv(gl_id(shader), model_view_loc(shader), &mv_mat_data[0]);
  const Matrix4f gl_mv_mat(mv_mat_data);

  // Construct the OpenGl model view matrix by hand.
  const Matrix4f X_CglC =
      (Matrix4f() << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1)
          .finished();
  const Eigen::DiagonalMatrix<float, 4, 4> scale_mat(
      Vector4<float>(scale(0), scale(1), scale(2), 1.0));
  const Matrix4f expected_mv_mat = X_CglC * X_CM * scale_mat;
  EXPECT_TRUE(CompareMatrices(expected_mv_mat, gl_mv_mat));
}

// Confirms that the clone contains the same data (including any derived data).
// This implicitly confirms that ShaderProgram::Clone calls DoClone().
TEST_F(ShaderProgramTest, Cloning) {
  TestShader shader{13};
  shader.LoadFromSources(kVertexSource, kFragmentSource);
  ShaderProgram* shader_ptr = &shader;
  auto shader_clone = dynamic_pointer_cast<TestShader>(shader_ptr->Clone());
  ASSERT_NE(shader_clone, nullptr);
  EXPECT_TRUE(Equals(*shader_clone, shader));
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
