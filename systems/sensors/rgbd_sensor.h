#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {
/** An RGB-D sensor that provides RGB, depth, and label images using the
 geometry in the geometry::SceneGraph.

 @system{RgbdSensor,
    @input_port{geometry_query},
    @output_port{color_image}
    @output_port{depth_image_32f}
    @output_port{depth_image_16u}
    @output_port{label_image}
    @output_port{X_WB}
 }

 Let `W` be the world coordinate system. In addition to `W`, there are three
 more coordinate systems that are associated with an %RgbdSensor. They are
 defined as follows (relative to the image):

   * `B` - the camera's base coordinate system: `X-forward`, `Y-left`, and
           `Z-up`.

   * `C` - the camera's color sensor's optical coordinate system: `X-right`,
           `Y-down` and `Z-forward`.

   * `D` - the camera's depth sensor's optical coordinate system: `X-right`,
           `Y-down` and `Z-forward`.

 Frames `C` and `D` are coincident and aligned. The origins of `C` and `D`
 (`Co` and `Do`, respectively) have position
 `p_BoCo_B = p_BoDo_B = <0 m, 0.02 m, 0 m>` by default. In other words
 `X_CD = I`. This definition implies that the depth image is a "registered depth
 image" for the RGB image, and that no disparity between the RGB and depth
 images are modeled in this system by default. For more details about the poses
 of `C` and `D`, see the class documentation of CameraInfo. These
 poses can be overwritten after construction with the appropriate methods
 (though you'll often only want to change their origins while keeping both
 sensors facing in the same direction).
 <!-- TODO(gizatt): The setters for modifying the sensor poses create a
 vulnerability that allows users to modify internal system state during
 simulation via a non-intended path. See PR#10491 for discussion;
 solutions could include enshrining these poses as proper parameters
 or accepting these poses during construction.-->

 Output port image formats:

   - color_image: Four channels, each channel uint8_t, in the following order:
     red, green, blue, and alpha.

   - depth_image_32f: One channel, float, representing the Z value in
     `D` in *meters*. The values 0 and infinity are reserved for out-of-range
     depth returns (too close or too far, respectively, as defined by
     DepthCameraProperties).

   - depth_image_16u: One channel, uint16_t, representing the Z value in
     `D` in *millimeters*. The values 0 and 65535 are reserved for out-of-range
     depth returns (too close or too far, respectively, as defined by
     DepthCameraProperties). Additionally, 65535 will also be returned if the
     depth measurement exceeds the representation range of uint16_t. Thus, the
     maximum valid depth return is 65534mm.

   - label_image: One channel, int16_t, whose value is a unique RenderLabel
     value. See RenderLabel for discussion of interpreting rendered labels.

 @note These depth sensor measurements differ from those of range data used by
 laser range finders (like DepthSensor), where the depth value represents the
 distance from the sensor origin to the object's surface.

 @ingroup sensor_systems  */
class RgbdSensor final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdSensor)

  // TODO(SeanCurtis-TRI): Seriously consider a OpenGL-style position-target-up
  //  constructor for RgbdSensor -- it can work for either stationary or
  //  generally affixed sensor as long as the position, target, and up vectors
  //  are all defined w.r.t. the parent frame.

  /** Constructs an %RgbdSensor whose frame `B` is rigidly affixed to the frame
   P, indicated by `frame_id`, and with the given camera properties. The camera
   will move as frame P moves. For a stationary camera, use the frame id from
   SceneGraph::world_frame_id().

   @param name           The name of the %RgbdSensor. This can be any value, but
                         typically should be unique.
   @param frame_id       The identifier of a parent frame `P` in
                         geometry::SceneGraph to which this camera is rigidly
                         affixed with pose `X_PB`.
   @param X_PB           The pose of the camera `B` frame relative to the parent
                         frame `P`.
   @param properties     The properties which define this camera's intrinsics.
   @param show_window    A flag for showing a visible window. If this is false,
                         off-screen rendering is executed. The default is false.
   */
  RgbdSensor(std::string name, geometry::FrameId parent_frame,
             const math::RigidTransformd& X_PB,
             const geometry::render::DepthCameraProperties& properties,
             bool show_window = false);

  ~RgbdSensor() = default;

  /** Returns the color sensor's info.  */
  const CameraInfo& color_camera_info() const { return color_camera_info_; }

  /** Returns the depth sensor's info.  */
  const CameraInfo& depth_camera_info() const { return depth_camera_info_; }

  /** Returns `X_BC`.  */
  const math::RigidTransformd& color_camera_optical_pose() const {
    return X_BC_;
  }

  /** Sets `X_BC`.  */
  void set_color_camera_optical_pose(const math::RigidTransformd& X_BC) {
    X_BC_ = X_BC;
  }

  /** Returns `X_BD`.  */
  const math::RigidTransformd& depth_camera_optical_pose() const {
    return X_BD_;
  }

  /** Sets `X_BD`.  */
  void set_depth_camera_optical_pose(const math::RigidTransformd& X_BD) {
    X_BD_ = X_BD;
  }

  /** Returns the id of the frame to which this camera is affixed.  */
  geometry::FrameId parent_frame_id() const { return parent_frame_; }

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

  // TODO(SeanCurtis-TRI): Investigate changing this simply to a RigidTransform.
  // Currently left as a PoseVector for backwards-compatibility reasons. After
  // all, there is only a single pose. Why wrap it in a PoseVector?
  /** Returns the abstract-valued output port that contains a PoseVector.  */
  const OutputPort<double>& sensor_base_pose_output_port() const;

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
                rendering::PoseVector<double>* pose_vector) const;

  // Convert a single channel, float depth image (with depths in meters) to a
  // single channel, unsigned uint16_t depth image (with depths in millimeters).
  static void ConvertDepth32FTo16U(const ImageDepth32F& d32,
                                   ImageDepth16U* d16);

  const geometry::QueryObject<double>& get_query_object(
      const Context<double>& context) const {
    return query_object_input_port().
        Eval<geometry::QueryObject<double>>(context);
  }

  const InputPort<double>* query_object_input_port_{};
  const OutputPort<double>* color_image_port_{};
  const OutputPort<double>* depth_image_32F_port_{};
  const OutputPort<double>* depth_image_16U_port_{};
  const OutputPort<double>* label_image_port_{};
  const OutputPort<double>* sensor_base_pose_port_{};

  // The identifier for the parent frame `P`.
  const geometry::FrameId parent_frame_;

  // If true, a window will be shown for the camera.
  const bool show_window_;
  const CameraInfo color_camera_info_;
  const CameraInfo depth_camera_info_;
  const geometry::render::DepthCameraProperties properties_;
  // The position of the camera's B frame relative to its parent frame P.
  const math::RigidTransformd X_PB_;

  // By default, the color sensor's origin (`Co`) is offset by 0.02 m on the Y
  // axis of the RgbdSensor's base coordinate system (`B`).
  //
  // The rotation R_BC can be written as:
  //      cx   cy   cz
  // bx   0    0    1
  // by  -1    0    0
  // bz   0   -1    0
  //
  // Reading the columns downward, we see that `cx` is in the `-by` direction;
  // this corresponds to the definition that `by` is "left" and `cx` is "right".
  // Similarly, `cy` is in the `-bz` direction. Again, by definition, `bz` is
  // "up" and `cy` is "down", so they point in opposite directions. Finally,
  // `cz` is in the direction of `bx`. `bx` and `cz` are both defined as
  // "forward", so they should point in the same direction.
  //
  // Do not attempt to achieve this via RPY or axis-angle representations; the
  // calculations invariably introduce small epsilon values into what should
  // otherwise be a perfectly represented matrix.
  math::RigidTransformd X_BC_{
    math::RotationMatrixd::MakeFromOrthonormalRows(
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(-1, 0, 0),
        Eigen::Vector3d(0, -1, 0)),
      Eigen::Vector3d(0, 0.02, 0)};

  math::RigidTransformd X_BD_{X_BC_};
};

/**
 Wraps a continuous %RgbdSensor with a zero-order hold to create a discrete
 sensor.

 @system{%RgbdSensorDiscrete,
    @input_port{geometry_query},
    @output_port{color_image}
    @output_port{depth_image_32f}
    @output_port{depth_image_16u}
    @output_port{label_image}
    @output_port{X_WB}
 }
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

  /** Returns mutable reference to RgbdSensor instance.  */
  RgbdSensor& mutable_sensor() { return *camera_; }

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

  /** @see RgbdSensor::sensor_base_pose_output_port().  */
  const systems::OutputPort<double>& sensor_base_pose_output_port() const {
    return get_output_port(output_port_pose_);
  }

 private:
  RgbdSensor* const camera_{};
  const double period_{};

  int query_object_port_{-1};
  int output_port_color_image_{-1};
  int output_port_depth_image_32F_{-1};
  int output_port_depth_image_16U_{-1};
  int output_port_label_image_{-1};
  int output_port_pose_{-1};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
