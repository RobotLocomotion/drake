#pragma once

#include <memory>
#include <optional>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {

#ifndef DRAKE_DOXYGEN_CXX
class RgbdSensor;
#endif

/** Wraps a continuous RgbdSensor with a zero-order hold to create a discrete
sensor.

@system
name: RgbdSensorDiscrete
input_ports:
- geometry_query
output_ports:
- color_image (optional)
- depth_image_32f (optional)
- depth_image_16u (optional)
- label_image (optional)
- body_pose_in_world
- image_time
@endsystem

See also RgbdSensorAsync for a slightly different discrete sensor model.

@note Be mindful that the discrete dynamics of a zero-order hold mean that all
selected image types (color, depth, and/or label) are rendered at the given
`fps`, even if nothing ends up using the images on the output ports; this might
be computationally wasteful. Be sure to only enable the image ports that you
actually need.

@ingroup sensor_systems  */
class RgbdSensorDiscrete final : public Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdSensorDiscrete);

  static constexpr double kDefaultPeriod = 1.0 / 30;

  /** Constructs a sensor with the given parameters. Refer to the class overview
  documentation for additional exposition and examples of the parameters.
  @param parent_id The frame "P" to which the sensor body is affixed.
    See RgbdSensor for details.
  @param X_PB The transform relating parent and sensor body.
    See RgbdSensor for details.
  @param fps How often (frames per second) to sample the `geometry_query` input
    and render the associated images. Must be strictly positive and finite.
  @param capture_offset The time of the first sample of `geometry_query` input.
    Typically zero. Must be non-negative and finite.
  @param color_camera The properties for the `color_image` output port.
    When nullopt, there will be no `color_image` output port.
    At least one of `color_camera` or `depth_camera` must be provided.
  @param depth_camera The properties for the `depth_image_32f` and
    `depth_image_16u` output ports.
    When nullopt, there will be no such output ports.
    At least one of `color_camera` or `depth_camera` must be provided.
  @param render_label_image Whether to provide the `label_image` output port.
    May be set to true only when a `color_camera` has also been provided. */
  RgbdSensorDiscrete(
      geometry::FrameId parent_id, const math::RigidTransformd& X_PB,
      double fps, double capture_offset,
      const std::optional<geometry::render::ColorRenderCamera>& color_camera,
      const std::optional<geometry::render::DepthRenderCamera>& depth_camera =
          {},
      bool render_label_image = false);

  // TODO(jwnimmer-tri) Mark this deprecated on 2023-08-01 or so.
  // At the same time, deprecate kDefaultPeriod.
  /** (To be deprecated) Constructs a diagram containing a (non-registered)
  RgbdSensor that will update at a given rate.
  @param sensor               The continuous sensor used to generate periodic
                              images.
  @param period               Update period (sec).
  @param render_label_image   If true, renders label image (which requires
                              additional overhead). If false,
                              `label_image_output_port` will raise an error if
                              called. */
  RgbdSensorDiscrete(std::unique_ptr<RgbdSensor> sensor,
                     double period = kDefaultPeriod,
                     bool render_label_image = true);

  /** Returns a reference to the underlying RgbdSensor object. */
  const RgbdSensor& sensor() const { return *camera_; }

  /** Returns the update period for the discrete camera. */
  double period() const { return period_; }

  /** Returns the update rate (frames per second) for the discrete camera. */
  double fps() const { return fps_; }

  /** Returns the `capture_offset` passed to the constructor. */
  double capture_offset() const { return capture_offset_; }

  /** Returns the geometry::QueryObject<double>-valued input port.  */
  const InputPort<double>& query_object_input_port() const {
    return get_input_port(query_object_input_port_);
  }

  /** Returns the abstract-valued output port that contains an ImageRgba8U.
  @throws std::exception when color output is not enabled. */
  const OutputPort<double>& color_image_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageDepth32F.
  @throws std::exception when depth output is not enabled. */
  const OutputPort<double>& depth_image_32F_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageDepth16U.
  @throws std::exception when depth output is not enabled. */
  const OutputPort<double>& depth_image_16U_output_port() const;

  /** Returns the abstract-valued output port that contains an ImageLabel16I.
  @throws std::exception when label output is not enabled. */
  const OutputPort<double>& label_image_output_port() const;

  /** Returns the abstract-valued output port (containing a RigidTransform)
  which reports the pose of the body in the world frame (X_WB). */
  const OutputPort<double>& body_pose_in_world_output_port() const {
    return get_output_port(body_pose_in_world_output_port_);
  }

  /** Returns the vector-valued output port (with size == 1) which reports the
  simulation time when the image outputs were captured, in seconds (i.e., the
  time of the most recent zero-order hold discrete update). */
  const OutputPort<double>& image_time_output_port() const {
    return get_output_port(image_time_output_port_);
  }

 private:
  // Both public constructors delegate to here.
  RgbdSensorDiscrete(std::unique_ptr<RgbdSensor> sensor, bool full_sized_empty,
                     double period, double fps, double capture_offset,
                     bool render_color_image, bool render_depth_image,
                     bool render_label_image);

  RgbdSensor* const camera_;
  // We store period and fps so that we can report back exactly the number that
  // was passed to our constructor. Once we remove the constructor that takes a
  // period, then we can nix the period_ here.
  const double period_;
  const double fps_;
  const double capture_offset_;

  int query_object_input_port_{-1};
  std::optional<int> color_image_output_port_;
  std::optional<int> depth_image_32F_output_port_;
  std::optional<int> depth_image_16U_output_port_;
  std::optional<int> label_image_output_port_;
  int body_pose_in_world_output_port_{-1};
  int image_time_output_port_{-1};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
