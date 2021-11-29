#pragma once

#include <string>
#include <utility>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace drake {
namespace examples {
namespace mesh_painter {

/** The %MaskImageCamera is similar to the RgbdSensor. However, it is
 specialized to work with a RenderEngineMaskImage instance so that the the
 image on this system's input port can be used as a mask in the color and label
 images.

 This system only allows a single geometry to use the mask. It is specified by
 its GeometryId. See systems::RgbdSensor for details on the camera semantics.
 */
class MaskImageCamera final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MaskImageCamera);
  /** Constructs an %MaskImageCamera whose frame `B` is rigidly affixed to the
   frame P, indicated by `parent_id`, and with the given camera properties. The
   camera will move as frame P moves. For a stationary camera, use the frame id
   from SceneGraph::world_frame_id().

   @param parent_id      The identifier of a parent frame `P` in
                         geometry::SceneGraph to which this camera is rigidly
                         affixed with pose `X_PB`.
   @param X_PB           The pose of the camera `B` frame relative to the parent
                         frame `P`.
   @param render_engine_name  The name of the render engine which will draw the
                         masked geometry.
   @param masked_id      The id of the geometry for whom the mask applies.
   @param color_properties Defines camera's color (and label) intrinsics and
                           renderer.
   @param depth_properties Defines camera's depth intrinsics and renderer.
   @param show_window    A flag for showing a visible window. If this is false,
                         off-screen rendering is executed. The default is false.

   @pydrake_mkdoc_identifier{color_props}
   */
  MaskImageCamera(
      geometry::FrameId parent_id,
      const math::RigidTransformd& X_PB, std::string render_engine_name,
      const geometry::GeometryId masked_id,
      const geometry::render::ColorRenderCamera& color_properties,
      const geometry::render::DepthRenderCamera& depth_properties);

  /** Constructs an %RgbdSensor in the same way as the above overload, but
   using the `RenderCameraCore` portion of `properties` for color (and label)
   properties, and all of `properties` for depth properties.

   @pydrake_mkdoc_identifier{use_depth_props}
   */
  MaskImageCamera(
      geometry::FrameId parent_id,
      const math::RigidTransformd& X_PB, std::string render_engine_name,
      const geometry::GeometryId masked_id,
      const geometry::render::DepthRenderCamera& properties,
      bool show_color_window = false)
      : MaskImageCamera(parent_id, X_PB, std::move(render_engine_name),
                        masked_id,
                        geometry::render::ColorRenderCamera(
                            properties.core(), show_color_window),
                        properties) {}

  ~MaskImageCamera() = default;

  // TODO(eric.cousineau): Expose which renderer color / depth uses?

  /** Returns the color sensor's info.  */
  const systems::sensors::CameraInfo& color_camera_info() const {
    return color_properties_.core().intrinsics();
  }

  /** Returns the depth sensor's info.  */
  const systems::sensors::CameraInfo& depth_camera_info() const {
    return depth_properties_.core().intrinsics();
  }

  /** Returns `X_BC`.  */
  const math::RigidTransformd& X_BC() const {
    return color_properties_.core().sensor_pose_in_camera_body();
  }

  /** Returns `X_BD`.  */
  const math::RigidTransformd& X_BD() const {
    return depth_properties_.core().sensor_pose_in_camera_body();
  }

  /** Returns the id of the frame to which the base is affixed.  */
  geometry::FrameId parent_frame_id() const { return parent_frame_id_; }

  /** Returns the geometry::QueryObject<double>-valued input port.  */
  const systems::InputPort<double>& query_object_input_port() const;

  /** Returns the abstract-valued input port for the mask texture.  */
  const systems::InputPort<double>& mask_texture_input_port() const;

  /** Returns the abstract-valued output port that contains an ImageRgba8U.  */
  const systems::OutputPort<double>& color_image_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageDepth32F.
   */
  const systems::OutputPort<double>& depth_image_32F_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageDepth16U.
   */
  const systems::OutputPort<double>& depth_image_16U_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageLabel16I.
   */
  const systems::OutputPort<double>& label_image_output_port() const;

  const systems::OutputPort<double>&
  camera_body_pose_in_world_output_port() const;

 private:
  enum MaskType { kColor, kLabel };

  void UpdateMaskImage(const systems::Context<double>& context,
                       MaskType mask_type) const;

  systems::EventStatus InitMaskedGeometry(
      const systems::Context<double>& context,
      systems::State<double>*) const;

  // The calculator methods for the four output ports.
  void CalcColorImage(const systems::Context<double>& context,
                      systems::sensors::ImageRgba8U* color_image) const;
  void CalcDepthImage32F(
      const systems::Context<double>& context,
      systems::sensors::ImageDepth32F* depth_image) const;
  void CalcDepthImage16U(
      const systems::Context<double>& context,
      systems::sensors::ImageDepth16U* depth_image) const;
  void CalcLabelImage(
      const systems::Context<double>& context,
      systems::sensors::ImageLabel16I* label_image) const;

  void CalcX_WB(
      const systems::Context<double>& context,
      math::RigidTransformd* X_WB) const;

  // Convert a single channel, float depth image (with depths in meters) to a
  // single channel, unsigned uint16_t depth image (with depths in millimeters).
  static void ConvertDepth32FTo16U(
      const systems::sensors::ImageDepth32F& d32,
      systems::sensors::ImageDepth16U* d16);

  // Extract the query object from the given context (via the appropriate input
  // port.
  const geometry::QueryObject<double>& get_query_object(
      const systems::Context<double>& context) const;

  const systems::InputPort<double>* query_object_input_port_{};
  const systems::InputPort<double>* mask_texture_input_port_{};
  const systems::OutputPort<double>* color_image_port_{};
  const systems::OutputPort<double>* depth_image_32F_port_{};
  const systems::OutputPort<double>* depth_image_16U_port_{};
  const systems::OutputPort<double>* label_image_port_{};
  const systems::OutputPort<double>*
      camera_body_pose_in_world_output_port_{};

  // The identifier for the parent frame `P`.
  const geometry::FrameId parent_frame_id_;

  // If true, a window will be shown for the camera.
  const geometry::render::ColorRenderCamera color_properties_;
  const geometry::render::DepthRenderCamera depth_properties_;
  // The position of the camera's B frame relative to its parent frame P.
  const math::RigidTransformd X_PB_;
  const std::string render_engine_name_;
  geometry::GeometryId masked_geometry_{};
};

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
