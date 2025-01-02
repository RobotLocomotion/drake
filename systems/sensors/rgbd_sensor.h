#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {

// TOOD(jwnimmer-tri) Maybe we should make this public at some point?  Is it
// helpful for users?
struct RgbdSensorParameters {
  // The default identifier for the parent frame `P`.
  geometry::FrameId parent_frame_id;
  // The camera specification for color & label.
  geometry::render::ColorRenderCamera color_camera;
  // The camera specification for depth.
  geometry::render::DepthRenderCamera depth_camera;
  // The position of the camera's B frame relative to its parent frame P.
  math::RigidTransformd X_PB;
};

}  // namespace internal

/** A meta-sensor that houses RGB, depth, and label cameras, producing their
 corresponding images based on the contents of the geometry::SceneGraph.

 @system
 name: RgbdSensor
 input_ports:
 - geometry_query
 output_ports:
 - color_image
 - depth_image_32f
 - depth_image_16u
 - label_image
 - body_pose_in_world
 - image_time
 @endsystem

 This system models a continuous sensor, where the output ports reflect the
 instantaneous images observed by the sensor. In contrast, a discrete (sample
 and hold) sensor model might be a more suitable match for a real-world camera;
 for that case, see RgbdSensorDiscrete or RgbdSensorAsync.

 The following text uses terminology and conventions from CameraInfo. Please
 review its documentation.

 This class uses the following frames:

   - W - world frame
   - C - color camera frame, used for both color and label cameras to guarantee
     perfect registration between color and label images.
   - D - depth camera frame
   - B - sensor body frame. Approximately, the frame of the "physical" sensor
     that contains the color, depth, and label cameras. The contained cameras
     are rigidly fixed to B and X_WB is what is used to pose the sensor in the
     world (or, alternatively, X_PB where P is some parent frame for which X_WP
     is known).

 By default, frames B, C, and D are coincident and aligned. These can be
 changed using the `camera_poses` constructor parameter. Frames C and D are
 always rigidly affixed to the sensor body frame B. As documented in the
 @ref camera_axes_in_image "CameraInfo documentation", the color and depth
 cameras "look" in the positive Cz and Dz directions, respectively with the
 positive Cy and Dy directions pointing to the bottom of the image. If R_BC and
 R_BD are the identity rotation, we can apply the same reasoning to the body
 frame: look in the +Bz direction with the +By direction pointing down in the
 image. Only if the depth or color frames are re-oriented relative to the body
 does further reasoning need to be applied.

 Output port image formats:

   - color_image: Four channels, each channel uint8_t, in the following order:
     red, green, blue, and alpha.

   - depth_image_32f: One channel, float, representing the Z value in
     `D` in *meters*. The values 0 and infinity are reserved for out-of-range
     depth returns (too close or too far, respectively, as defined by
     @ref geometry::render::DepthRenderCamera "DepthRenderCamera").

   - depth_image_16u: One channel, uint16_t, representing the Z value in
     `D` in *millimeters*. The values 0 and 65535 are reserved for out-of-range
     depth returns (too close or too far, respectively, as defined by
     @ref geometry::render::DepthRenderCamera "DepthRenderCamera").
     Additionally, 65535 will also be returned if the
     depth measurement exceeds the representation range of uint16_t. Thus, the
     maximum valid depth return is 65534mm.

   - label_image: One channel, int16_t, whose value is a unique
     @ref geometry::render::RenderLabel "RenderLabel" value aligned with the
     color camera frame. See @ref geometry::render::RenderLabel "RenderLabel"
     for discussion of interpreting rendered labels.

 @note These depth sensor measurements differ from those of range data used by
 laser range finders (like DepthSensor), where the depth value represents the
 distance from the sensor origin to the object's surface.

 @ingroup sensor_systems  */
class RgbdSensor final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdSensor);

  /** Constructs an %RgbdSensor with fully specified render camera models for
   both color/label and depth cameras.
   @pydrake_mkdoc_identifier{individual_intrinsics}  */
  RgbdSensor(geometry::FrameId parent_id, const math::RigidTransformd& X_PB,
             geometry::render::ColorRenderCamera color_camera,
             geometry::render::DepthRenderCamera depth_camera);

  /** Constructs an %RgbdSensor using a fully specified depth render camera,
   inferring the color settings based on depth. The color camera in inferred
   from the `depth_camera`; it shares the same
   geometry::render::RenderCameraCore and is configured to show the window
   based on the value of `show_color_window`.
   @pydrake_mkdoc_identifier{combined_intrinsics} */
  RgbdSensor(geometry::FrameId parent_id, const math::RigidTransformd& X_PB,
             const geometry::render::DepthRenderCamera& depth_camera,
             bool show_color_window = false);

  ~RgbdSensor();

  /** Returns the default render camera for color/label renderings. */
  const geometry::render::ColorRenderCamera& default_color_render_camera()
      const;

  /** Sets the default render camera for color/label renderings. */
  void set_default_color_render_camera(
      const geometry::render::ColorRenderCamera& color_camera);

  /** Returns the context dependent render camera for color/label renderings. */
  const geometry::render::ColorRenderCamera& GetColorRenderCamera(
      const Context<double>& context) const;

  /** Sets the render camera for color/label renderings, as stored as
   parameters in `context`. */
  void SetColorRenderCamera(
      Context<double>* context,
      const geometry::render::ColorRenderCamera& color_camera) const;

  /** Returns the default render camera for depth renderings.  */
  const geometry::render::DepthRenderCamera& default_depth_render_camera()
      const;

  /** Sets the default render camera for depth renderings. */
  void set_default_depth_render_camera(
      const geometry::render::DepthRenderCamera& depth_camera);

  /** Returns the context dependent render camera for depth renderings. */
  const geometry::render::DepthRenderCamera& GetDepthRenderCamera(
      const Context<double>& context) const;

  /** Sets the render camera for depth renderings, as stored as parameters in
   `context`. */
  void SetDepthRenderCamera(
      Context<double>* context,
      const geometry::render::DepthRenderCamera& depth_camera) const;

  /** Returns the default `X_PB`.  */
  const math::RigidTransformd& default_X_PB() const;

  /** Sets the default `X_PB`.  */
  void set_default_X_PB(const math::RigidTransformd& sensor_pose);

  /** Returns the context dependent `X_PB`. */
  const math::RigidTransformd& GetX_PB(const Context<double>& context) const;

  /** Sets the `X_PB`, as stored as parameters in `context`. */
  void SetX_PB(Context<double>* context,
               const math::RigidTransformd& sensor_pose) const;

  /** Returns the default id of the frame to which the body is affixed.  */
  geometry::FrameId default_parent_frame_id() const;

  /** Sets the default id of the frame to which the body is affixed.  */
  void set_default_parent_frame_id(geometry::FrameId id);

  /** Returns the context dependent id of the frame to which the body is
   affixed. */
  geometry::FrameId GetParentFrameId(const Context<double>& context) const;

  /** Sets the id of the frame to which the body is affixed, as stored as
   parameters in `context`. */
  void SetParentFrameId(Context<double>* context, geometry::FrameId id) const;

  /** Returns the geometry::QueryObject<double>-valued input port.  */
  const InputPort<double>& query_object_input_port() const;

  /** Returns the abstract-valued output port that contains an ImageRgba8U.  */
  const OutputPort<double>& color_image_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageDepth32F.
   */
  const OutputPort<double>& depth_image_32F_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageDepth16U.
   */
  const OutputPort<double>& depth_image_16U_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageLabel16I.
   */
  const OutputPort<double>& label_image_output_port() const;

  /** Returns the abstract-valued output port (containing a RigidTransform)
   which reports the pose of the body in the world frame (X_WB).  */
  const OutputPort<double>& body_pose_in_world_output_port() const;

  /** Returns the vector-valued output port (with size == 1) that reports the
   current simulation time, in seconds. This is provided for consistency with
   RgbdSensorDiscrete and RgbdSensorAsync (where the image time is not always
   the current time). */
  const OutputPort<double>& image_time_output_port() const;

 private:
  // The calculator methods for the four output ports.
  void CalcColorImage(const Context<double>& context,
                      ImageRgba8U* color_image) const;
  void CalcDepthImage32F(const Context<double>& context,
                         ImageDepth32F* depth_image) const;
  void CalcDepthImage16U(const Context<double>& context,
                         ImageDepth16U* depth_image) const;
  void CalcLabelImage(const Context<double>& context,
                      ImageLabel16I* label_image) const;
  void CalcX_WB(const Context<double>& context,
                math::RigidTransformd* X_WB) const;
  void CalcImageTime(const Context<double>&, BasicVector<double>*) const;

  // Writes the current default values to the context's parameters.
  void SetDefaultParameters(const Context<double>& context,
                            Parameters<double>* parameters) const override;

  // Extracts the query object from the given context (via the appropriate
  // input port.
  const geometry::QueryObject<double>& get_query_object(
      const Context<double>& context) const;

  const InputPort<double>* query_object_input_port_{};
  const OutputPort<double>* color_image_port_{};
  const OutputPort<double>* depth_image_32F_port_{};
  const OutputPort<double>* depth_image_16U_port_{};
  const OutputPort<double>* label_image_port_{};
  const OutputPort<double>* body_pose_in_world_output_port_{};
  const OutputPort<double>* image_time_output_port_{};

  internal::RgbdSensorParameters defaults_;
  AbstractParameterIndex parameter_index_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake

// This exists for backwards compatibility reasons. The discrete sensor class
// was previously defined within this file.
#include "drake/systems/sensors/rgbd_sensor_discrete.h"
