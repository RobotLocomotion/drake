#pragma once

#include <string>
#include <vector>

#include "robotlocomotion/image_array_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace systems {
namespace sensors {

/// An ImageToLcmImageArrayT takes as input an ImageRgba8U, ImageDepth32F and
/// ImageLabel16I. This system outputs an AbstractValue containing a
/// `Value<robotlocomotion::image_array_t>` LCM message that defines an array
/// of images (image_t). This message can then be sent to other processes that
/// sbscribe it using LcmPublisherSystem. Note that you should NOT assume any
/// particular order of those images stored in robotlocomotion::image_array_t,
/// instead check the semantic of those images with
/// robotlocomotion::image_t::pixel_format before using them.
class ImageToLcmImageArrayT : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageToLcmImageArrayT)

  /// An %ImageToLcmImageArrayT constructor.  Declares three input ports --
  /// one color image, one depth image, and one label image.
  ///
  /// @param color_frame_name The frame name used for color image.
  /// @param depth_frame_name The frame name used for depth image.
  /// @param label_frame_name The frame name used for label image.
  /// @param do_compress When true, zlib compression will be performed. The
  /// default is false.
  ImageToLcmImageArrayT(const std::string& color_frame_name,
                        const std::string& depth_frame_name,
                        const std::string& label_frame_name,
                        bool do_compress = false);

  /// Returns the input port containing a color image.
  /// Note: Only valid if the color/depth/label constructor is used.
  const InputPort<double>& color_image_input_port() const;

  /// Returns the input port containing a depth image.
  /// Note: Only valid if the color/depth/label constructor is used.
  const InputPort<double>& depth_image_input_port() const;

  /// Returns the input port containing a label image.
  /// Note: Only valid if the color/depth/label constructor is used.
  const InputPort<double>& label_image_input_port() const;

  /// Returns the abstract valued output port that contains a
  /// `Value<robotlocomotion::image_array_t>`.
  const OutputPort<double>& image_array_t_msg_output_port() const;

  /// Default constructor doesn't declare any ports.  Use the Add*Input()
  /// methods to declare them.
  explicit ImageToLcmImageArrayT(bool do_compress = false);

  template <PixelType kPixelType>
  const InputPort<double>& DeclareImageInputPort(const std::string& name) {
    input_port_pixel_type_.push_back(kPixelType);
    return this->DeclareAbstractInputPort(
        name, Value<Image<kPixelType>>());
  }

 private:
  void CalcImageArray(const systems::Context<double>& context,
                      robotlocomotion::image_array_t* msg) const;

  int color_image_input_port_index_{-1};
  int depth_image_input_port_index_{-1};
  int label_image_input_port_index_{-1};
  int image_array_t_msg_output_port_index_{-1};

  std::vector<PixelType> input_port_pixel_type_{};
  const bool do_compress_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
