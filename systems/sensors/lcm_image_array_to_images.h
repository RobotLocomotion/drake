#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/// An LcmImageArrayToImages takes as input an AbstractValue containing a
/// `Value<lcmt_image_array>` LCM message that defines an array
/// of images (lcmt_image).  The system has output ports for one color image as
/// an ImageRgba8U and one depth image as ImageDepth32F (intended to be
/// similar to the API of RgbdCamera, though without the label image port).
class LcmImageArrayToImages : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmImageArrayToImages)

  LcmImageArrayToImages();

  // TODO(jwnimmer-tri) The "_t" or "T" suffix on this method name is
  // superfluous and should be removed.
  /// Returns the abstract valued input port that expects a
  /// `Value<lcmt_image_array>`.
  const InputPort<double>& image_array_t_input_port() const {
    return this->get_input_port(image_array_t_input_port_index_);
  }

  /// Returns the abstract valued output port that contains a RGBA image of
  /// the type ImageRgba8U.  The image will be empty if no color image was
  /// received in the most recent message (so, for example, sending color and
  /// depth in different messages will not produce useful results).
  const OutputPort<double>& color_image_output_port() const {
    return this->get_output_port(color_image_output_port_index_);
  }

  /// Returns the abstract valued output port that contains an ImageDepth32F.
  /// The image will be empty if no color image was received in the most
  /// recent message (so, for example, sending color and depth in different
  /// messages will not produce useful results).
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
