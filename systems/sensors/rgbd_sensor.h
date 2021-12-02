#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

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
 @endsystem

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdSensor)

  /** Constructs an %RgbdSensor with fully specified render camera models for
   both color/label and depth cameras.
   @pydrake_mkdoc_identifier{individual_intrinsics}  */
  RgbdSensor(geometry::FrameId parent_id, const math::RigidTransformd& X_PB,
             geometry::render::ColorRenderCamera color_camera,
             geometry::render::DepthRenderCamera depth_camera);

  /** Constructs an %RgbdSensor with fully specified render camera models for
   both the depth camera. The color camera in inferred from the `depth_camera`;
   it shares the same geometry::render::RenderCameraCore and is configured to
   show the window based on the value of `show_color_window`.
   @pydrake_mkdoc_identifier{combined_intrinsics}  */
  RgbdSensor(geometry::FrameId parent_id, const math::RigidTransformd& X_PB,
             const geometry::render::DepthRenderCamera& depth_camera,
             bool show_color_window = false);

  ~RgbdSensor() = default;

  // TODO(eric.cousineau): Expose which renderer color / depth uses?

  // TODO(SeanCurtis-TRI): Deprecate this in favor of the color render camera.
  /** Returns the intrinsics properties of the color camera model.  */
  const CameraInfo& color_camera_info() const {
    return color_camera_.core().intrinsics();
  }

  // TODO(SeanCurtis-TRI): Deprecate this in favor of the depth render camera.
  /** Returns the intrinsics properties of the depth camera model.  */
  const CameraInfo& depth_camera_info() const {
    return depth_camera_.core().intrinsics();
  }

  /** Returns the render camera for color/label renderings.  */
  const geometry::render::ColorRenderCamera& color_render_camera() const {
    return color_camera_;
  }

  /** Returns the render camera for depth renderings.  */
  const geometry::render::DepthRenderCamera& depth_render_camera() const {
    return depth_camera_;
  }

  /** Returns `X_BC`.  */
  const math::RigidTransformd& X_BC() const {
    return color_camera_.core().sensor_pose_in_camera_body();
  }

  /** Returns `X_BD`.  */
  const math::RigidTransformd& X_BD() const {
    return depth_camera_.core().sensor_pose_in_camera_body();
  }

  /** Returns the id of the frame to which the base is affixed.  */
  geometry::FrameId parent_frame_id() const { return parent_frame_id_; }

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

 private:
  friend class RgbdSensorTester;

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

  // Convert a single channel, float depth image (with depths in meters) to a
  // single channel, unsigned uint16_t depth image (with depths in millimeters).
  static void ConvertDepth32FTo16U(const ImageDepth32F& d32,
                                   ImageDepth16U* d16);

  // Extract the query object from the given context (via the appropriate input
  // port.
  const geometry::QueryObject<double>& get_query_object(
      const Context<double>& context) const;

  const InputPort<double>* query_object_input_port_{};
  const OutputPort<double>* color_image_port_{};
  const OutputPort<double>* depth_image_32F_port_{};
  const OutputPort<double>* depth_image_16U_port_{};
  const OutputPort<double>* label_image_port_{};
  const OutputPort<double>* body_pose_in_world_output_port_{};

  // The identifier for the parent frame `P`.
  const geometry::FrameId parent_frame_id_;

  // The camera specifications for color/label and depth.
  const geometry::render::ColorRenderCamera color_camera_;
  const geometry::render::DepthRenderCamera depth_camera_;
  // The position of the camera's B frame relative to its parent frame P.
  const math::RigidTransformd X_PB_;
};

/**
 Wraps a continuous %RgbdSensor with a zero-order hold to create a discrete
 sensor.

 @system
 name: RgbdSensorDiscrete
 input_ports:
 - geometry_query
 output_ports:
 - color_image
 - depth_image_32f
 - depth_image_16u
 - label_image
 - body_pose_in_world
 @endsystem
 */
class RgbdSensorDiscrete final : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdSensorDiscrete);

  static constexpr double kDefaultPeriod = 1. / 30;

  /** Constructs a diagram containing a (non-registered) RgbdSensor that will
   update at a given rate.
   @param sensor               The continuous sensor used to generate periodic
                               images.
   @param period               Update period (sec).
   @param render_label_image   If true, renders label image (which requires
                               additional overhead). If false,
                               `label_image_output_port` will raise an error if
                               called.  */
  RgbdSensorDiscrete(std::unique_ptr<RgbdSensor> sensor,
                     double period = kDefaultPeriod,
                     bool render_label_image = true);

  /** Returns reference to RgbdSensor instance.  */
  const RgbdSensor& sensor() const { return *camera_; }

  /** Returns update period for discrete camera.  */
  double period() const { return period_; }

  /** @see RgbdSensor::query_object_input_port().  */
  const InputPort<double>& query_object_input_port() const {
    return get_input_port(query_object_port_);
  }

  /** @see RgbdSensor::color_image_output_port().  */
  const systems::OutputPort<double>& color_image_output_port() const {
    return get_output_port(output_port_color_image_);
  }

  /** @see RgbdSensor::depth_image_32F_output_port().  */
  const systems::OutputPort<double>& depth_image_32F_output_port() const {
    return get_output_port(output_port_depth_image_32F_);
  }

  /** @see RgbdSensor::depth_image_16U_output_port().  */
  const systems::OutputPort<double>& depth_image_16U_output_port() const {
    return get_output_port(output_port_depth_image_16U_);
  }

  /** @see RgbdSensor::label_image_output_port().  */
  const systems::OutputPort<double>& label_image_output_port() const {
    return get_output_port(output_port_label_image_);
  }

  /** @see RgbdSensor::body_pose_in_world_output_port().  */
  const OutputPort<double>& body_pose_in_world_output_port() const {
    return get_output_port(body_pose_in_world_output_port_);
  }

 private:
  RgbdSensor* const camera_{};
  const double period_{};

  int query_object_port_{-1};
  int output_port_color_image_{-1};
  int output_port_depth_image_32F_{-1};
  int output_port_depth_image_16U_{-1};
  int output_port_label_image_{-1};
  int body_pose_in_world_output_port_{-1};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
