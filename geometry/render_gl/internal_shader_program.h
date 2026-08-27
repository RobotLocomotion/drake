#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/light_parameter.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gl/internal_opengl_includes.h"
#include "drake/geometry/render_gl/internal_shader_program_data.h"
#include "drake/geometry/rgba.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

/* TODO(SeanCurtis-TRI) This implies a compile-time shader definition. It may
 be preferred to move away from this architecture. To do runtime shader
 configuration requires careful coordination between the runtime configurable
 properties and the compile-time mechanisms. Plenty of guidance and
 documentation on what constitutes valid declarations. (I.e., you have to assume
 that vertex position is at location 0, etc.)

 Furthermore, the definition of the shader must be accompanied by consistent
 shader *meta data* which will inform the ShaderProgram class of what uniforms,
 pixel attributes, etc., are required, their types, and where they come from.
 This last -- where they come from -- is going to be compile-time constrained.
 There should be an enumerated number of quantities (of known types) that
 ShaderProgram can bind.

 Alternatively, we could ape the VTK architecture with the exception being
 that we require *declaration* of shaders (or provide automatic clustering of
 identical shaders) such that we don't have a ridiculous proliferation of
 shaders, leading to a thrashing OpenGL context. This last option has the
 benefit of providing cross-RenderEngine shader compatibility. All we need is
 the ability to manufacture a shader and can pass that into any pipeline that
 consumes glsl (and knows how to set properties/uniforms). */

/* Definition of a GLSL shader program including a vertex and fragment shader.
 All operations on this shader program require an active OpenGL context. In
 fact, they require the same context which was active when the program was
 "loaded".

 Explicit functionality is given in derived classes. They specify the actual
 shader code and any idiosyncratic details. To be compatible with *this*
 shader abstraction, a shader must specify a uniform mat4 called "T_DM" for the
 model-to-device matrix. This is a projective transform taking a model-frame
 point in ℜ³ to homogeneous OpenGL clip coordinates. We define this matrix
 as:

   `T_DM = T_DCgl * T_CglCPhysical * X_CphysicalW * T_WM`

 where,

   - `T_WM`:  the model-to-world transform (see documentation on frames for
      OpenGlGeometry).
   - `X_CphysicalW`: the rigid relationship between the physical camera frame
     and the world frame. This is passed to SetModelViewMatrix() as X_CW.
   - `T_CglCPhysical`: the fixed transform between Drake's physical camera frame
     OpenGL's camera frame. The two cameras have different conventions. This is
     hard-coded in the shader program's architecture.
   - `T_DCgl`: the OpenGL projection matrix. This is passed to
      SetProjectionMatrix() as T_DCgl.
   - We define `T_DCphysical = T_DCgl * T_CglCPhysical`. This is a constant for
     the camera's intrinsics and is what is *internally* stored as a result of
     calling SetProjectionMatrix().

 The %ShaderProgram stores the identifier for an OpenGl program. These programs
 are included in the objects that are shared across OpenGl contexts. However,
 when rendering in parallel, this sharing can be problematic. Setting uniforms'
 values in one thread can leak through to the other threads, possibly changing
 the rendered output. To make sure that one RenderEngineGl instance rendering
 in one thread doesn't interfere with its clone in another thread, the clone
 should call Relink() on each of its ShaderProgram instances with its own
 context bound during the clone's construction -- this guarantees an independent
 namespace for its uniforms (at the cost of increased GPU memory). Relink() is
 not called automatically as part of the cloning, because the OpenGl context for
 the cloned %ShaderProgram would have to be bound at the time of cloning. */
class ShaderProgram {
 public:
  ShaderProgram() : id_(ShaderId::get_new_id()) {}

  /* The destructor is currently superficial. The real work is done by Free()
   (see below for details).  */
  virtual ~ShaderProgram();

  std::unique_ptr<ShaderProgram> Clone() const { return DoClone(); }

  /* Loads a %ShaderProgram from GLSL code contained in the provided strings.

   @param vertex_shader_source    The valid GLSL source code for a vertex
                                  shader.
   @param fragment_shader_source  The valid GLSL source code for a fragment
                                  shader.
   @throws std::exception if either shader source doesn't compile or the
                          resulting program has errors.
   */
  void LoadFromSources(const std::string& vertex_shader_source,
                       const std::string& fragment_shader_source);

  /* Loads a %ShaderProgram from GLSL code contained in the named files.

   @param vertex_shader_file    The path to a file containing valid GLSL source
                                code for a vertex shader.
   @param fragment_shader_file  The path to a file containing valid GLSL source
                                code for a fragment shader.
   @throws std::exception if there is an error in reading the files, either
                          shader source doesn't compile, or the resulting
                          program has errors.
   */
  void LoadFromFiles(const std::string& vertex_shader_file,
                     const std::string& fragment_shader_file);

  /* Each %ShaderProgram can examine a set of properties and determine if it
   the properties are sufficient to apply this shader to the geometry with the
   given `properties`. This data will be used by the shader at render time to
   configure the per-instance shader parameters.

   The derived classes define the details in DoCreateProgramData.

   @returns The validated and packaged shader program properties, `nullopt` if
            `this` shader program cannot be applied for the given properties.
   @pre The OpenGl context to which this program belongs has been bound. */
  std::optional<ShaderProgramData> CreateProgramData(
      const PerceptionProperties& properties) const {
    return DoCreateProgramData(properties);
  }

  /* Allows derived shaders to extract data from the given instance to populate
   *per-instance* shader parameters. */
  virtual void SetInstanceParameters(
      const ShaderProgramData& /* data */) const {}

  /* Allows derived shaders to manipulate OpenGl state based on camera
   properties. This should *not* include model -> camera -> device transforms.
   they are handled elsewhere.  */
  virtual void SetDepthCameraParameters(
      const render::DepthRenderCamera& /* camera */) const {}

  /* Stores T_DCphysical by multiplying the given `T_DCgl` with the internal
   `T_CglCPhysical` transform. Only needs to be called once per unique set of
   camera intrinsics (doesn't depend on rendered geometry poses or even camera
   pose). */
  void SetProjectionMatrix(const Eigen::Matrix4f& T_DCgl) const;

  /* Stores the `T_DM`, the OpenGL model-to-device matrix. Allows derived
   shader programs to do any other instance- and camera-dependent
   configuration).

   This method invokes DoSetModelViewMatrix() with many of the component
   matrices.

   Note: there is a subtle difference between the geometry frame G and the model
   frame M. Geometries are typically defined in a canonical frame in which they
   have unit dimensions. This canonical geometry is scaled to create the model.
   Therefore, the geometry frame G and model frame M are related by the scale
   matrix S_MG (such that it is zero off the diagonal and has the x-, y-, and
   z-scale values along the diagonal).

   @param X_CW   The transform relating the world frame and the camera's
                 physical frame. Noted above as X_CphysicalW.
   @param T_WM   The transform to map the model's vertex positions into the
                 world frame. Not necessarily a rigid transform.
   @param N_WM   The transform to map the model's vertex normals into the
                 world frame.
   @pre SetProjectionMatrix() has been called with the current camera's
        projection matrix (unchecked). */
  void SetModelViewMatrix(const Eigen::Matrix4f& X_CW,
                          const Eigen::Matrix4f& T_WM,
                          const Eigen::Matrix3f& N_WM) const;

  /* Provides the location of the named shader uniform parameter.
   @throws std::exception if the named uniform isn't part of the program. */
  GLint GetUniformLocation(const std::string& uniform_name) const;

  /* Binds the program for usage.  */
  void Use() const;

  /* Unbinds the program (if this is the current program). Derived classes are
   not obliged to call this. RenderEngineGl promises to only call operations
   on a particular ShaderProgram after calling its Use() method.  */
  void Unuse() const;

  /* Reports the Drake identifier for this shader; this is not the OpenGl
   identifier used in OpenGl APIs.  */
  ShaderId shader_id() const { return id_; }

  /* TODO(SeanCurtis-TRI) This "relink" after cloning is a primitive solution to
   the problem of unique uniform namespaces. I tried using program pipeline
   objects but ended up with a worse outcome (might be programmer error). I have
   yet to explore Uniform Buffer Objects -- this should be explored so that the
   programs don't have to be relinked. Instead, each clone of ShaderProgram
   would simply have a unique buffer object to guarantee thread independence. */

  /* Constructs a new OpenGl program based on the already compiled shaders.

   If `this` instance was cloned from another instance, this will create a
   unique uniform namespace from the source instance. This is necessary if the
   two instances are going to be exercised in parallel.

   @pre `this` is a clone of a fully functional %ShaderProgram (i.e., the
        shaders have been compiled and linked and we are ready to call Use()).
   @pre The OpenGl context that possesses the compiled shaders is bound. */
  void Relink();

  /* Deletes the OpenGl program from the currently bound context. This should
   be a precursor to destroying the %ShaderProgram.

   Ideally, this would be done in the destructor. However, the RenderEngineGl
   infrastructure is not clearly wired to guarantee that the invocation of
   shader program destructors is preconditioned by a bound context.

   Furthermore, it renders the destructor really brittle. The destructor only
   "works" if someone outside the destructor has taken a special action. As long
   as that is the requirement, that actor might as well couple *that* special
   act with *this* special clean up. */
  void Free();

 protected:
  /* Extracts uniform ids from the linked program. After extracting the common
   uniform identifiers, it invokes DoConfigureUniforms() so derived class can
   extract their particular uniform ids. */
  void ConfigureUniforms();

  /* Invoked after successfully linking an OpenGl program. Derived classes
   should use this to extract the ids of the program's unique uniform variables.
   */
  virtual void DoConfigureUniforms() = 0;

  /* The copy and move semantics are only made available for sub-classes so
   they can easily implement DoClone.  */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ShaderProgram);

  /* Derived classes should clone themselves.  */
  virtual std::unique_ptr<ShaderProgram> DoClone() const = 0;

  /* Derived classes define if the properties support the use of this shader.
   Not supported is communicated with a nullopt. Support is given by a defined
   ShaderProgramData (even if it's data *value* is nullptr).  */
  virtual std::optional<ShaderProgramData> DoCreateProgramData(
      const PerceptionProperties& /* properties */) const {
    return std::nullopt;
  }

  /* Derived classes can set additional state based on model instance
   transforms.

   @param X_CW     The relative transform between camera and world frames. This
                   camera frame is different than the Cgl frame used in the
                   actual model view matrix, C is the physical frame and Cgl
                   accounts for OpenGL conventions.
   @param T_WM     The transform relating the model and world frames. For a
                   model with no scale (and no poses of its constituent parts),
                   this would be equal to the X_WG pose provided by SceneGraph.
   @param N_WM     Derived from T_WM, it transforms the model's normals into the
                   world frame.  */
  virtual void DoSetModelViewMatrix(const Eigen::Matrix4f& /* X_CW */,
                                    const Eigen::Matrix4f& /* T_WM */,
                                    const Eigen::Matrix3f& /* N_WM */) const {}

 private:
  friend class ShaderProgramTest;

  // The Drake identifier for this shader. It exists whether the shader has been
  // correctly compiled or not.
  ShaderId id_;

  GLuint gl_id_{0};

  // The OpenGl identifiers for the vertex and fragment shaders; these are saved
  // to facilitate copying the shader program (saving compilation time).
  GLuint vertex_id_{0};
  GLuint fragment_id_{0};

  // The physical-camera-to-device matrix is common to all instances rendered
  // by this shader. It is cached and combined with each instance's model-view
  // matrix.
  mutable Eigen::Matrix4f T_DCphysical_{Eigen::Matrix4f::Identity()};

  // Location of the model-to-device matrix in the supported shader.
  GLint model_to_device_matrix_loc_{};
};

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
