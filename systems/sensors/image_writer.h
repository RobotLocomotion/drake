#pragma once

/** @file Provides utilities for writing images to disk.

 This file provides two sets of utilities: stand alone methods that can be
 invoked in any context and a System that can be connected into a diagram to
 automatically capture images during simulation at a fixed frequency.  */

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/** @name     Utility functions for writing common image types to disk.

 Given a fully-specified path to the file to write and corresponding image data,
 these functions will _attempt_ to write the image data to the file. The
 functions assume that the path is valid and writable.  */
//@{

/** Writes the color (8-bit, RGBA) image data to disk.  */
void SaveToPng(const std::string& file_path, const ImageRgba8U& image);

/** Writes the depth (32-bit) image data to disk. Png files do not support
 channels larger than 16-bits and its support for floating point values is
 also limited at best. So, depth images can only be written as tiffs.  */
void SaveToTiff(const std::string& file_path, const ImageDepth32F& image);

/** Writes the label (16-bit) image data to disk.  */
void SaveToPng(const std::string& file_path, const ImageLabel16I& image);

//@}

/** A system for periodically writing images to the file system. This is a
 convenience mechanism for capturing images from (typically) an RgbdCamera.
 However, any system that outputs a compatible image type can act as an input
 for this system.

 @system{ImageWriter,
    @input_port{color_image} @input_port{depth_image} @input_port{label_image},
 }

 The writer saves images based on a periodic publish event. For a given period
 `P`, images will be written at times `t = [0, P, 2P, ...]`. Optionally, the
 writer can be configured to create images starting at some time greater than
 zero. Given a positive "start time" value `tₛ`, the first frame will be written
 at `t = minᵢ i⋅P`, such that `i ∈ ℕ, t ≥ tₛ` -- not necessarily exactly _at_
 `tₛ`.

 The output image names are a combination of a provided base name, the image
 type, and an optionally zero-padded frame enumeration. The enumeration starts
 at 0. For example, given a base name of "my_image", padding of 3, the
 %ImageWriter could produce image files with the following names:

   - `my_image_color_002.png`: the _third_ color image written.
   - `my_image_depth_020.tiff`: the _21ˢᵗ_ depth image written.
   - `my_image_label_1000.png`: the 1001ˢᵗ label image written.

 Each image type corresponds to a unique input port. If the input port is not
 connected, that image type will not be written. All image types are written
 to the same directory. All image types are written at the same frequency and
 images of different types with the same enumeration value reflect the same
 time represented in the Context. If different image types need to be written at
 different periods or to different directories, simply instantiate multiple
 %ImageWriter instances and configure them accordingly.

 @note Color and label images are saved as png files -- a lossless compressed
 image format. However, depth images are stored as tiff files; png does not
 support 32-bit floating-point-valued channels.

 @note Every invocation of `Publish()` that writes images will increment the
 frame counter. If `Publish()` is only called based on this system's periodic
 events (see set_publish_period()`), then the image sequence will directly map
 to uniform intervals of time. Calls to `Publish()` outside of the periodic
 events will render enumerated frames into the sequence without this same
 relationship in time.  */
class ImageWriter : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageWriter)

  /** Constructs the image-to-file system. The system writes *files* but does
   not create directories. If the given `folder_path` specifies a directory
   that does not exist (or can't be written to), the system will be disabled.
   A warning will be emitted but, otherwise, it will not interfere with the
   evaluation of the diagram in which this image writer is placed.

   @param folder_path The path to the directory into which the images will be
                      written. The empty string is interpreted as "/".
   @param image_name  The base name of the files to write. Images will be
                      written as `[image_name]_[type]_%0Xd.ext` where X is
                      `padding`. Defaults to "image".
   @param start_time  The earliest simulation time at which an image may be
                      written. See class documentation on the time at which
                      the first frame will be written relative to `start_time`.
                      Defaults to zero.
   @param padding     The minimum number of digits in the output file name for
                      image sequence enumeration (with the total count padded by
                      zeros). For example:
                        padding = 1 --> image1.png, image2.png, ...
                        padding = 2 --> image01.png, image02.png, ...
                        padding = 3 --> image001.png, image002.png, ...
                        etc.
                      Default value is 4.  */
  ImageWriter(std::string folder_path, std::string image_name = "image",
            double start_time = 0, int padding = 4);

  /** The period at which images are written. See the class documentation for
   details regarding the time stamps at which images will be written. `period`
   must be positive.
   @throws std::logic_error if period is not positive.  */
  void set_publish_period(double period);

  /** @name     Image-typed input ports      */
  //@{

  const InputPort<double>& color_image_input_port() const;

  const InputPort<double>& depth_image_input_port() const;

  const InputPort<double>& label_image_input_port() const;

  //@}

 private:
#ifndef DRAKE_DOXYGEN_CXX
  // Friend for facilitating unit testing.
  friend class ImageWriterTester;
#endif

  // Returns true if the directory path provided is valid: it exists, it's a
  // directory, and it's writable.
  static bool IsDirectoryValid(const std::string& file_path);

  // Creates a file name from the writer's base name, image directory, and the
  // given type name and frame number.
  std::string make_file_name(const std::string& type_name,
                             int frame_number, const std::string& ext) const;

  // Do the image writing.
  void DoPublish(
      const systems::Context<double>& context,
      const std::vector<const PublishEvent<double>*>&) const override;

  // Output parameters.
  const std::string folder_path_;
  const std::string image_name_base_;
  const double start_time_{0.0};
  const int padding_{4};

  const bool can_write_{false};

  int color_image_port_index_{-1};
  int depth_image_port_index_{-1};
  int label_image_port_index_{-1};

  // The index of the _last_ frame created. For simplicity's sake, this is
  // simply a mutable member because it is *only* touched in the `Publish()`
  // method -- that means it won't be affected by a Simulator's test steps
  // or an integrator's intermediate steps. The alternative of putting it in
  // discrete state feels overly heavyweight for such a simple use.
  mutable int previous_frame_index_{-1};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
