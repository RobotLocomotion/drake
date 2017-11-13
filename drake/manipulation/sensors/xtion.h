#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace sensors {

/**
 * Attaches an Asus Xtion PRO Live camera to a given frame on a RigidBodyTree,
 * and provides the ability to add the camera as a system in a diagram builder.
 *
 * Frames:
 *  B - RgbdCamera sensor frame (X-forward, Y-left, Z-up).
 *  X - Xtion base frame
 *      Same orientation as RgbdCamera frame, but with different offset.
 *      To visualize base pose, run `meshlab xtion.obj`, go to
 *      Render > Show Axis.
 *
 * @code{.cc}
 *  // Add to tree.
 *  auto* camera = new Xtion("xtion");
 *  // Add to tree.
 *  camera->AddToTree(&tree_builder, fixture_frame);
 *  // Build diagram.
 *  camera->BuildDiagram(&lcm, true, true);
 *  // Add to system.
 *  builder.AddSystem(std::unique_ptr<Xtion>(camera));
 *  builder.Connect(plant->get_output_port_state(),
 *                  camera->get_input_port_state());
 * @endcode
 */
// TODO(eric.cousineau): Delegate most of this class's functionality to
// definition files once sensor tags are supported. At present, this is
// merely meant to be a convenience wrapper.
class Xtion : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Xtion)

  // TODO(eric.cousineau): Add option for noise.

  /// @name Technical specifications
  ///@{
  static const double kFovY;
  static const double kDepthRangeNear;
  static const double kDepthRangeFar;
  ///@}

  /**
   * Initialize Xtion container.
   * @name name Name used in the model instance and the camera system instance.
   */
  explicit Xtion(const std::string& name);

  /**
   * Attach camera to the world with a given transform.
   * @param tree_builder Tree builder instance.
   * @param fixture_frame Frame to which the Xtion base (X) is to be attached.
   * @pre There must be one call only to this method.
   * @pre `BuildDiagram(...)` should not yet be called.
   */
  void AddToTree(util::WorldSimTreeBuilder<double>* tree_builder,
                 std::shared_ptr<RigidBodyFrame<double>> fixture_frame);

  /**
   * Build diagram containing RGBD camera with Xtion settings.
   * @param lcm LCM instance. This must not be null if either
   * `add_lcm_publisher` or `add_frame_visualizer` are true.
   * @param add_lcm_publisher Adds LCM publisher to the channel
   * DRAKE_RGBD_CAMERA_IMAGES.
   * @param add_frame_visualizer Adds LCM-based frame visualizer.
   * @pre `AddToTree(...)` must have been called.
   */
  void BuildDiagram(lcm::DrakeLcm* lcm = nullptr,
                    bool add_lcm_publisher = false,
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
   * @pre `BuildDiagram(...)` should not yet be called.
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
   * @pre `BuildDiagram(...)` should not yet be called.
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
