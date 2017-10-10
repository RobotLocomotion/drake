#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rgbd_camera.h"

namespace drake {
namespace manipulation {
namespace sensors {

/**
 * Attaches an Asus Xtion PRO Live camera to a given frame on a RigidBodyTree,
 * and provides the ability to add the camera as a system in a diagram builder.
 *
 * Frames:
 *  B - RgbdCamera sensor frame (X-forward, Y-left, Z-up).
 *  X - Xtion frame
 *      Same orientation as RgbdCamera frame, but with different offset.
 *      To visualize base pose, run `meshlab xtion.obj`, go to
 *      Render > Show Axis.
 *
 * @code{.cc}
 *  // Add to tree.
 *  auto* camera = new Xtion(&tree_builder, wsg_id);
 *  // Build diagram.
 *  camera->Build(&lcm, true, true);
 *  // Add to system.
 *  builder.AddSystem(std::unique_ptr<Xtion>(camera));
 *  builder.Connect(plant->get_output_port_state(),
 *                  camera->get_input_port_state());
 * @endcode
 */
// TODO(eric.cousineau): Delegate most of this class's functionality to
// definition files once sensor tags are supported.
class Xtion : public systems::Diagram<double> {
 public:
  // TODO(eric.cousineau): Add option for noise.

  // @ref https://www.asus.com/us/3D-Sensor/Xtion_PRO_LIVE/specifications/
  static constexpr double kFovY = M_PI_4;
  static constexpr double kDepthRangeNear = 0.8;
  static constexpr double kDepthRangeFar = 3.5;

  /**
   * Attach camera to a given body with a given transform.
   * @name name Name used in the model instance and the camera system instance.
   * @param tree_builder Tree builder instance.
   * @param fixture_frame The frame X to which the Xtion is attached.
   */
  Xtion(const std::string& name,
        util::WorldSimTreeBuilder<double>* tree_builder,
        std::shared_ptr<RigidBodyFrame<double>> fixture_frame);

  /**
   * Attach camera to the world with a given transform.
   * @name name Name used in the model instance and the camera system instance.
   * @param tree_builder Tree builder instance.
   * @param X_WX Pose of the Xtion frame X relative to the world frame W.
   */
  Xtion(const std::string& name,
        util::WorldSimTreeBuilder<double>* tree_builder,
        const Eigen::Isometry3d& X_WX);

  /**
   * Build diagram containing RGBD camera with Xtion settings.
   * @param lcm LCM instance. This must not be null of either
   * `add_lcm_publisher` or `add_frame_visualizer` are true.
   * @param add_lcm_publisher Adds LCM publisher to the channel
   * DRAKE_RGBD_CAMERA_IMAGES.
   * @param add_frame_visualizer Adds LCM-based frame visualizer.
   */
  void Build(lcm::DrakeLcm* lcm = nullptr, bool add_lcm_publisher = false,
             bool add_frame_visualizer = false);

  const systems::InputPortDescriptor<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  }
  const systems::OutputPort<double>& get_output_port_color_image() const {
    return get_output_port(output_port_color_image_);
  }
  const systems::OutputPort<double>& get_output_port_depth_image() const {
    return get_output_port(output_port_depth_image_);
  }
  const systems::OutputPort<double>& get_output_port_label_image() const {
    return get_output_port(output_port_label_image_);
  }

  /**
   * @return Period at which the camera captures images.
   * If there is a LCM publisher and/or a frame visualizer, then these will
   * publish on this same period.
   */
  double period() const { return period_; }
  /**
   * Sets period at which the camera captures images.
   * @pre `Build(...)` should not yet be called.
   */
  void set_period(double period) {
    DRAKE_DEMAND(!initialized_);
    period_ = period;
  }

  /**
   * @return LCM channel which images are published to.
   * @see ImageToLcmImageArrayT for information on types.
   */
  const std::string& lcm_channel() const { return lcm_channel_; }
  /**
   * Sets the LCM channel on which the camera images are published.
   * @pre `Build(...)` should not yet be called.
   */
  void set_lcm_channel(const std::string& lcm_channel) {
    DRAKE_DEMAND(!initialized_);
    lcm_channel_ = lcm_channel;
  }

 private:
  std::string name_;
  std::string lcm_channel_{"DRAKE_RGBD_CAMERA_IMAGES"};
  double period_{1. / 30};
  bool initialized_{false};

  const RigidBodyTree<double>* tree_{};
  std::shared_ptr<RigidBodyFrame<double>> fixture_frame_;
  std::shared_ptr<RigidBodyFrame<double>> sensor_frame_;

  int input_port_state_{-1};
  int output_port_color_image_{-1};
  int output_port_depth_image_{-1};
  int output_port_label_image_{-1};
};

}  // namespace sensors
}  // namespace manipulation
}  // namespace drake
