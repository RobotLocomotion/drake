#pragma once

#include <memory>

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
- color_image
- depth_image_32f
- depth_image_16u
- label_image
- body_pose_in_world
@endsystem

See also RgbdSensorAsync for a slightly different discrete sensor model.

@note Be mindful that the discrete dynamics of a zero-order hold mean that all
three image types (color, depth, label) are rendered at the given `period`,
even if nothing ends up using the images on the output ports; this might be
computationally wasteful. If you only need color and depth, be sure to pass
`render_label_image = false` to the constructor. If you only need one image
type, eschew this system in favor of adding your own zero-order hold on just
the one RgbdSensor output port that you need.

@ingroup sensor_systems  */
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
                              called. */
  RgbdSensorDiscrete(std::unique_ptr<RgbdSensor> sensor,
                     double period = kDefaultPeriod,
                     bool render_label_image = true);

  /** Returns reference to RgbdSensor instance. */
  const RgbdSensor& sensor() const { return *camera_; }

  /** Returns update period for discrete camera. */
  double period() const { return period_; }

  /** @see RgbdSensor::query_object_input_port(). */
  const InputPort<double>& query_object_input_port() const {
    return get_input_port(query_object_port_);
  }

  /** @see RgbdSensor::color_image_output_port(). */
  const systems::OutputPort<double>& color_image_output_port() const {
    return get_output_port(output_port_color_image_);
  }

  /** @see RgbdSensor::depth_image_32F_output_port(). */
  const systems::OutputPort<double>& depth_image_32F_output_port() const {
    return get_output_port(output_port_depth_image_32F_);
  }

  /** @see RgbdSensor::depth_image_16U_output_port(). */
  const systems::OutputPort<double>& depth_image_16U_output_port() const {
    return get_output_port(output_port_depth_image_16U_);
  }

  /** @see RgbdSensor::label_image_output_port(). */
  const systems::OutputPort<double>& label_image_output_port() const {
    return get_output_port(output_port_label_image_);
  }

  /** @see RgbdSensor::body_pose_in_world_output_port(). */
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
