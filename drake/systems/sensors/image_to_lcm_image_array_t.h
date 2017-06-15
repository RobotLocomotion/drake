#pragma once

#include <string>

#include "robotlocomotion/image_array_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace sensors {

/// An ImageToLcmImageArrayT takes as input a ImageBgra8U, ImageDepth32F and
/// ImageLabel16I. This system outputs an AbstractValue containing a
/// `Value<robotlocomotion::image_array_t>` LCM message that defines an array
/// of images (image_t). This message can then be sent to other processes that
/// sbscribe it using LcmPublisherSystem.
// TODO(kunimatsu-tri) Instead of assuming fixed pixel types for the input
// ports, e.g. ImageBgra8U, change the interface to be able to handle arbitrary
// pixel types of `Image`.
class ImageToLcmImageArrayT : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageToLcmImageArrayT)

  /// A %ImageToLcmImageArrayT constructor.
  ///
  /// @param color_frame_name The frame name used for color image.
  /// @param depth_frame_name The frame name used for depth image.
  /// @param label_frame_name The frame name used for label image.
  ImageToLcmImageArrayT(const std::string& color_frame_name,
                        const std::string& depth_frame_name,
                        const std::string& label_frame_name);

  /// Returns a descriptor of the input port containing a color image.
  const InputPortDescriptor<double>& color_image_input_port() const;

  /// Returns a descriptor of the input port containing a depth image.
  const InputPortDescriptor<double>& depth_image_input_port() const;

  /// Returns a descriptor of the input port containing a label image.
  const InputPortDescriptor<double>& label_image_input_port() const;

  /// Returns a descriptor of the abstract valued output port that contains a
  /// `Value<robotlocomotion::image_array_t>`.
  const OutputPort<double>& image_array_t_msg_output_port() const;

 private:
  void CalcImageArray(const systems::Context<double>& context,
                      robotlocomotion::image_array_t* msg) const;

  int color_image_input_port_index_{};
  int depth_image_input_port_index_{};
  int label_image_input_port_index_{};
  int image_array_t_msg_output_port_index_{};

  std::string color_frame_name_;
  std::string depth_frame_name_;
  std::string label_frame_name_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
