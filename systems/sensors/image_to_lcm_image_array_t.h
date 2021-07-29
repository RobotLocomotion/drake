#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_image_array.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace systems {
namespace sensors {

// TODO(jwnimmer-tri) Throughout this filename, classname, and method names, the
// the "_t" or "T" suffix is superfluous and should be removed.

/// An ImageToLcmImageArrayT takes as input an ImageRgba8U, ImageDepth32F and
/// ImageLabel16I. This system outputs an AbstractValue containing a
/// `Value<lcmt_image_array>` LCM message that defines an array of images
/// (lcmt_image). This message can then be sent to other processes that
/// sbscribe it using LcmPublisherSystem. Note that you should NOT assume any
/// particular order of those images stored in lcmt_image_array,
/// instead check the semantic of those images with
/// lcmt_image::pixel_format before using them.
///
/// @note The output message's header field `seq` is always zero.
class ImageToLcmImageArrayT : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageToLcmImageArrayT)

  /// Constructs an empty system with no input ports.
  /// After construction, use DeclareImageInputPort() to add inputs.
  explicit ImageToLcmImageArrayT(bool do_compress = false);

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
  /// `Value<lcmt_image_array>`.
  const OutputPort<double>& image_array_t_msg_output_port() const;

  template <PixelType kPixelType>
  const InputPort<double>& DeclareImageInputPort(const std::string& name) {
    input_port_pixel_type_.push_back(kPixelType);
    return this->DeclareAbstractInputPort(
        name, Value<Image<kPixelType>>());
  }

 private:
  void CalcImageArray(const systems::Context<double>& context,
                      lcmt_image_array* msg) const;

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
