#pragma once

#include <optional>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/** A sensor similar to RgbdSensorDiscrete but the rendering occurs on a
background thread to offer improved performance.

@experimental

@warning This system is intended for use only with the out-of-process glTF
rendering engine (MakeRenderEngineGltfClient()). Other render engines (e.g.,
MakeRenderEngineVtk() or MakeRenderEngineGl()) are either not thread-safe or
perform poorly when used here. We hope to add async support for those engines
down the road (see #19437 for details).

@system
name: RgbdSensorAsync
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

This sensor works by capturing the i'th state of the world at time
`t_capture = capture_offset + i / fps`
and outputting the corresponding rendered image at time
`t_output = t_capture + output_delay`,
i.e.,
`t_output = capture_offset + i / fps + output_delay`.

For example, with the following constructor settings:
- capture_offset = 0 ms
- output_delay = 50 ms
- fps = 5 Hz (i.e., 200 ms)

We have the following timing schedule:

 | Time   | Event   | Current output images    |
 | :----: | :-----: | :----------------------: |
 |   0 ms | capture | empty (i.e., zero-sized) |
 |  50 ms | output  | the scene as of 0 ms     |
 | 200 ms | capture | ^                        |
 | 250 ms | output  | the scene as of 200 ms   |
 | 400 ms | capture | ^                        |
 | ...    | ...     | ...                      |

The `capture_offset` is typically zero, but can be useful to stagger multiple
cameras in the same scene to trigger at different times if desired (e.g., to
space out rendering events for better performance).

The `output_delay` can be used to model delays of a physical camera (e.g., for
firmware processing on the device or transmitting the image over a USB or
network connection, etc.).

The image rendering between the capture event and the output event happens on a
background thread, allowing simulation time to continue to advance during the
rendering's `output_delay`. This helps smooth over the runtime latency
associated with rendering.

See also RgbdSensorDiscrete for a simpler (unthreaded) discrete sensor model, or
RgbdSensor for a continuous model.

@warning As the moment, this implementation cannnot respond to changes to
geometry shapes, textures, etc. after a simulation has started. The only thing
it responds to are changes to geometry poses. If you change anything beyond
poses, you must manually call Simulator::Initialize() to reset things before
resuming the simulation, or else you'll get an exception. This holds true for
changes to both SceneGraph's model geometry and the copy of the geometry in a
scene graph Context. */
class RgbdSensorAsync final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdSensorAsync);

  /** Constructs a sensor with the given parameters. Refer to the class overview
  documentation for additional exposition and examples of the parameters.
  @param scene_graph The SceneGraph to use for image rendering. This pointer is
    retained by the constructor so must remain valid for the lifetime of this
    object. After construction, this system's `geometry_query` input port must
    be connected to the relevant output port of the given `scene_graph`.
  @param parent_id The frame "P" to which the sensor body is affixed.
    See RgbdSensor for details.
  @param X_PB The transform relating parent and sensor body.
    See RgbdSensor for details.
  @param fps How often (frames per second) to sample the `geometry_query` input
    and render the associated images. Must be strictly positive and finite.
  @param capture_offset The time of the first sample of `geometry_query` input.
    Typically zero. Must be non-negative and finite.
  @param output_delay How long after the `geometry_query` input sample the
    output ports should change to reflect the new rendered image(s).
    Must be strictly positive and strictly less than 1/fps.
  @param color_camera The properties for the `color_image` output port.
    When nullopt, there will be no `color_image` output port.
    At least one of `color_camera` or `depth_camera` must be provided.
  @param depth_camera The properties for the `depth_image_32f` and
    `depth_image_16u` output ports.
    When nullopt, there will be no such output ports.
    At least one of `color_camera` or `depth_camera` must be provided.
  @param render_label_image Whether to provide the `label_image` output port.
    May be set to true only when a `color_camera` has also been provided. */
  RgbdSensorAsync(
      const geometry::SceneGraph<double>* scene_graph,
      geometry::FrameId parent_id, const math::RigidTransformd& X_PB,
      double fps, double capture_offset, double output_delay,
      std::optional<geometry::render::ColorRenderCamera> color_camera,
      std::optional<geometry::render::DepthRenderCamera> depth_camera = {},
      bool render_label_image = false);

  /** Returns the `parent_id` passed to the constructor. */
  geometry::FrameId parent_id() const { return parent_id_; }

  /** Returns the `X_PB` passed to the constructor. */
  const math::RigidTransformd& X_PB() const { return X_PB_; }

  /** Returns the `fps` passed to the constructor. */
  double fps() const { return fps_; }

  /** Returns the `capture_offset` passed to the constructor. */
  double capture_offset() const { return capture_offset_; }

  /** Returns the `output_delay` passed to the constructor. */
  double output_delay() const { return output_delay_; }

  /** Returns the `color_camera` passed to the constructor. */
  const std::optional<geometry::render::ColorRenderCamera>& color_camera()
      const {
    return color_camera_;
  }

  /** Returns the `depth_camera` passed to the constructor. */
  const std::optional<geometry::render::DepthRenderCamera>& depth_camera()
      const {
    return depth_camera_;
  }

  // TODO(jwnimmer-tri) Add an output port for the timestamp associated with
  // the other output ports (images, etc.) to make it easier for the user to
  // know when any given image was captured.

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
  const OutputPort<double>& body_pose_in_world_output_port() const;

  /** Returns the vector-valued output port (with size == 1) which reports the
  simulation time when the image outputs were captured, in seconds. When there
  are no images available on the output ports (e.g., at the beginning of a
  simulation), the value will be NaN. */
  const OutputPort<double>& image_time_output_port() const;

 private:
  struct TickTockState;

  const TickTockState& get_state(const Context<double>&) const;
  TickTockState& get_mutable_state(State<double>*) const;

  EventStatus Initialize(const Context<double>&, State<double>*) const;
  void CalcTick(const Context<double>&, State<double>*) const;
  void CalcTock(const Context<double>&, State<double>*) const;
  void CalcColor(const Context<double>&, ImageRgba8U*) const;
  void CalcLabel(const Context<double>&, ImageLabel16I*) const;
  void CalcDepth32F(const Context<double>&, ImageDepth32F*) const;
  void CalcDepth16U(const Context<double>&, ImageDepth16U*) const;
  void CalcX_WB(const Context<double>&, math::RigidTransformd*) const;
  void CalcImageTime(const Context<double>&, BasicVector<double>*) const;

  const geometry::SceneGraph<double>* const scene_graph_;
  const geometry::FrameId parent_id_;
  const math::RigidTransformd X_PB_;
  const double fps_;
  const double capture_offset_;
  const double output_delay_;
  const std::optional<geometry::render::ColorRenderCamera> color_camera_;
  const std::optional<geometry::render::DepthRenderCamera> depth_camera_;
  const bool render_label_image_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
