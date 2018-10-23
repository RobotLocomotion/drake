#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/// An LcmImageArrayToImages takes as input an AbstractValue containing a
/// `Value<robotlocomotion::image_array_t>` LCM message that defines an array
/// of images (image_t).  The system has output ports for one color image as
/// an ImageRgba8U and one depth image as ImageDepth32F (intended to be
/// similar to the API of RgbdCamera, though without the label image port).
class LcmImageArrayToImages : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmImageArrayToImages)

  /// An %ImageToLcmImageArrayT constructor.
  ///
  /// @param color_frame_name The frame name used for color image.
  /// @param depth_frame_name The frame name used for depth image.
  /// @param label_frame_name The frame name used for label image.
  /// @param do_compress When true, zlib compression will be performed. The
  /// default is false.
  LcmImageArrayToImages();

  /// Returns the abstract valued input port that expects a
  /// `Value<robotlocomotion::image_array_t>`.
  const InputPort<double>& image_array_t_input_port() const {
    return this->get_input_port(image_array_t_input_port_index_);
  }

  /// Returns the abstract valued output port that contains a RGBA image of the
  /// type ImageRgba8U.
  const OutputPort<double>& color_image_output_port() const {
    return this->get_output_port(color_image_output_port_index_);
  }

  /// Returns the abstract valued output port that contains an ImageDepth32F.
  const OutputPort<double>& depth_image_output_port() const {
    return this->get_output_port(depth_image_output_port_index_);
  }

 private:
  void CalcColorImage(const Context<double>& context,
                      ImageRgba8U* color_image) const;
  void CalcDepthImage(const Context<double>& context,
                      ImageDepth32F* depth_image) const;

  const InputPortIndex image_array_t_input_port_index_{};
  const OutputPortIndex color_image_output_port_index_{};
  const OutputPortIndex depth_image_output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
