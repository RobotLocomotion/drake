#pragma once

#include <string>
#include <utility>

#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {

/* Specification of an RGB-D sensor: its sensor properties, how it connects into
 the plant, and publication configuration. */
class SimRgbdSensor {
 public:
  // TODO(sean-curtis): The name "serial" has some issues.
  //  - "Serial" as a noun, doesn't mean serial number. Strictly speaking, its
  //    actual meaning is inappropriate for this context.
  //  - In practice, it's merely a suffix that gets applied to LCM channels
  //    and system names. It need not be a "serial number". It could be any
  //    value convenient to the author of the model.
  //  - The main interface (CameraConfig) has the field `name` which gets
  //    mapped to serial in its construction of SimRgbdSensor. So, we have
  //    "name" vs "serial" consistency problems.
  //  We should pick and document the name that we want with intention.
  /*
    RgbdSensor is a meta sensor that contains multiple cameras. Please refer
    to `drake/systems/sensors/rgbd_sensor.h` for more details.

    @param serial           The identifier for the sensor - this is used in the
                            name of the RgbdSensor as well as to provide names
                            for the image LCM channels.
    @param frame_P          The parent frame P in MultibodyPlant the sensor is
                            affixed to. This frame gets aliased by `this`.
    @param rate_hz          The publishing rate for the sensor.
    @param X_PB             The pose of the sensor frame B expressed with
                            respect to its parent frame P.
    @param color_properties The properties of the color camera.
    @param depth_properties The properties of the depth camera.

    @warning Note that frame P may not have a *direct* analog in a SceneGraph;
    instead, SceneGraph will have a frame for body A (the body that parent frame
    P is attached to). In this case, usages must account for X_AP when adding a
    camera. For example, see the implementation of `AddSimRgbdSensor`.
   */
  SimRgbdSensor(const std::string& serial,
                const multibody::Frame<double>& frame_P, double rate_hz,
                const math::RigidTransformd& X_PB,
                const geometry::render::ColorRenderCamera& color_properties,
                const geometry::render::DepthRenderCamera& depth_properties)
      : serial_(serial),
        frame_(&frame_P),
        rate_hz_(rate_hz),
        X_PB_(X_PB),
        color_properties_(color_properties),
        depth_properties_(depth_properties) {}

  const std::string& serial() const { return serial_; }

  const multibody::Frame<double>& frame() const { return *frame_; }

  const math::RigidTransform<double>& X_PB() const { return X_PB_; }

  const geometry::render::ColorRenderCamera& color_properties() const {
    return color_properties_;
  }

  const geometry::render::DepthRenderCamera& depth_properties() const {
    return depth_properties_;
  }

  double rate_hz() const { return rate_hz_; }

 private:
  std::string serial_;
  const multibody::Frame<double>* frame_{};
  double rate_hz_{};
  math::RigidTransformd X_PB_;
  geometry::render::ColorRenderCamera color_properties_;
  geometry::render::DepthRenderCamera depth_properties_;
};

// TODO(eric.cousineau): Ew... Is there a way to not need the plant?
/* Adds an RGB-D camera, as described by `sim_camera`, to the diagram.
 @param scene_graph       The SceneGraph that supplies the rendered image for
                          the camera.
 @param plant             The plant that owns the frame the sensor will be
                          affixed to.
 @param sim_camera        The description of the camera.
 @param[in,out] builder   The new RgbdSensor will be added to this builder's
                          diagram.
 @pre The description in `sim_camera` is compatible with the configuration in
      SceneGraph (e.g., camera properties refer to an existing RenderEngine).
 */
RgbdSensor* AddSimRgbdSensor(const geometry::SceneGraph<double>& scene_graph,
                             const multibody::MultibodyPlant<double>& plant,
                             const SimRgbdSensor& sim_camera,
                             DiagramBuilder<double>* builder);

/* Helper function that converts a plant frame with offset (per SimRgbdSensor)
to a geometry frame with offset (per RgbdSensor).
@param X_PB is relative to the plant_frame "P". */
std::pair<geometry::FrameId, math::RigidTransformd> GetGeometryFrame(
    const multibody::Frame<double>& plant_frame,
    const math::RigidTransformd& X_PB);

/* Adds LCM publishers for images (RGB, depth, and/or label), at the camera's
  specified rate.

 @param serial           The SimRgbdSensor.serial().
 @param fps              The SimRgbdSensor.rate_hz().
 @param publish_offset   Phase offset for publishing; see LcmPublisherSystem
                         for a comprehensive description.
 @param rgb_port         The (optional) port for color images.
 @param depth_16u_port   The (optional) port for depth images.
 @param label_port       The (optional) port for label images.
 @param do_compress      If true, the published images will be compressed.
 @param[in,out] builder  The publishing infrastructure will be added to this
                         builder's diagram.
 @param[in,out] lcm      The lcm interface to use. This interface object must
                         remain alive at least as long as the systems added
                         to `builder`.

 @pre lcm != nullptr.
 @pre builder != nullptr.
 @pre rgb_port is null or ImageRgba8U-valued.
 @pre depth_16u_port is null or ImageDepth16U-valued.
 @pre label_port is null or ImageLabel16I-valued. */
void AddSimRgbdSensorLcmPublisher(std::string_view serial, double fps,
                                  double publish_offset,
                                  const OutputPort<double>* rgb_port,
                                  const OutputPort<double>* depth_16u_port,
                                  const OutputPort<double>* label_port,
                                  bool do_compress,
                                  DiagramBuilder<double>* builder,
                                  drake::lcm::DrakeLcmInterface* lcm);

// A backwards-compatibility overload to simplify unit testing.
void AddSimRgbdSensorLcmPublisher(const SimRgbdSensor& sim_camera,
                                  const OutputPort<double>* rgb_port,
                                  const OutputPort<double>* depth_16u_port,
                                  const OutputPort<double>* label_port,
                                  bool do_compress,
                                  DiagramBuilder<double>* builder,
                                  drake::lcm::DrakeLcmInterface* lcm);

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
