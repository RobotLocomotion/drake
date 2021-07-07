#pragma once

/** @file Provides utilities for writing images to disk.

 This file provides two sets of utilities: stand alone methods that can be
 invoked in any context and a System that can be connected into a diagram to
 automatically capture images during simulation at a fixed frequency.  */

#include <string>
#include <unordered_map>
#include <utility>
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
 functions assume that the path is valid and writable. These functions will
 attempt to write the image to the given file path. The file format will be
 that indicated by the function name, but the extension will be whatever is
 provided as input.

 These function do not do validation on the provided file path (existence,
 writability, correspondence with image type, etc.) It relies on the caller to
 have done so.  */
//@{

/** Writes the color (8-bit, RGBA) image data to disk.  */
void SaveToPng(const ImageRgba8U& image, const std::string& file_path);

/** Writes the depth (32-bit) image data to disk. Png files do not support
 channels larger than 16-bits and its support for floating point values is
 also limited at best. So, depth images can only be written as tiffs.  */
void SaveToTiff(const ImageDepth32F& image, const std::string& file_path);

/** Writes the label (16-bit) image data to disk.  */
void SaveToPng(const ImageLabel16I& image, const std::string& file_path);

/** Writes the depth (16-bit) image data to disk.  */
void SaveToPng(const ImageDepth16U& image, const std::string& file_path);

/** Writes the grey scale (8-bit) image data to disk.  */
void SaveToPng(const ImageGrey8U& image, const std::string& file_path);

//@}

/** A system for periodically writing images to the file system. The system does
 not have a fixed set of input ports; the system can have an arbitrary number of
 image input ports. Each input port is independently configured with respect to:

   - publish frequency,
   - write location (directory) and image name,
   - input image format (which, in turn, implies a file-system format),
   - port name (which needs to be unique across all input ports in this system),
     and
   - context time at which output starts.

 By design, this system is intended to work with RgbdCamera, but can connect to
 any output port that provides images.

 @system
 name: ImageWriter
 input_ports:
 - declared_image1
 - declared_image2
 - ...
 - declared_imageN
 @endsystem

 %ImageWriter supports three specific types of images:

   - ImageRgba8U - typically a color image written to disk as .png images.
   - ImageDepth32F - typically a depth image, written to disk as .tiff images.
   - ImageLabel16I - typically a label image, written to disk as .png images.
   - ImageDepth16U - typically a depth image, written to disk as .png images.
   - ImageGrey8U - typically a grey scale image, written to disk as .png images.

 Input ports are added to an %ImageWriter via DeclareImageInputPort(). See
 that function's documentation for elaboration on how to configure image output.
 It is important to note, that every declared image input port _must_ be
 connected; otherwise, attempting to write an image from that port, will cause
 an error in the system.  */
class ImageWriter : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageWriter)

  /** Constructs default instance with no image ports.  */
  ImageWriter();

  /** Declares and configures a new image input port. A port is configured by
   providing:

     - a unique port name,
     - an output file format string,
     - a publish period,
     - a start time, and
     - an image type.

   Each port is evaluated independently, so that two ports on the same
   %ImageWriter can write images to different locations at different
   frequencies, etc. If images are to be kept in sync (e.g., registered color
   and depth images), they should be given the same period and start time.

   <h3>Specifying the times at which images are written</h3>

   Given a _positive_ publish period `p`, images will be written at times
   contained in the list of times: `t = [0, 1⋅p, 2⋅p, ...]`. The start time
   parameter determines what the _first_ output time will be. Given a "start
   time" value `tₛ`, the frames will be written at:
   `t = tₛ + [0, 1⋅p, 2⋅p, ...]`.

   <h3>Specifying write location and output file names</h3>

   When writing image data to disk, the location and name of the output files
   are controlled by a user-defined format string. The format string should be
   compatible with `fmt::format()`. %ImageWriter provides several _named_ format
   arguments that can be referenced in the format string:

     - `port_name`   - The name of the port (see below).
     - `image_type`  - One of `color`, `depth`, or `label`, depending on the
                       image type requested.
     - `time_double` - The time (in seconds) stored in the context at the
                       invocation of Publish(), represented as a double.
     - `time_usec`   - The time (in microseconds) stored in the context at the
                       invocation of Publish(), represented as a 64-bit integer.
     - `time_msec`   - The time (in milliseconds) stored in the context at the
                       invocation of Publish(), represented as an integer.
     - `count`       - The number of images that have been written from this
                       port (the first image would get zero, the Nᵗʰ would get
                       N - 1). This value increments _every_ time an image gets
                       written.

   File names can then be specified as shown in the following examples (assuming
   the port was declared as a color image port, with a name of "my_port", a
   period of 0.02 s (50 Hz), and a start time of 5 s.

     - `/home/user/images/{port_name}/{time_usec}` creates a sequence like:
       - `/home/user/images/my_port/5000000.png`
       - `/home/user/images/my_port/5020000.png`
       - `/home/user/images/my_port/5040000.png`
       - ...
     - `/home/user/images/{image_type}/{time_msec:05}` creates a sequence like:
       - `/home/user/images/color/05000.png`
       - `/home/user/images/color/05020.png`
       - `/home/user/images/color/05040.png`
       - ...
     - `/home/user/{port_name}/my_image_{count:03}.txt` creates a sequence like:
       - `/home/user/my_port/my_image_000.txt.png`
       - `/home/user/my_port/my_image_001.txt.png`
       - `/home/user/my_port/my_image_002.txt.png`
       - ...

   We call attention particularly to the following:

     - Note the zero-padding arguments in the second and third examples. Making
       use of zero-padding typically facilitates _other_ processes.
     - If the file name format does not end with an appropriate extension (e.g.,
       `.png` or `.tiff`), the extension will be added.
     - The directory specified in the format will be tested for validity
       (does it exist, is it a directory, can the program write to it). The
       full _file name_ will _not_ be validated. If it is invalid (e.g., too
       long, invalid characters, bad format substitution), images will silently
       not be created.
     - The third example uses the count flag -- regardless of start time, the
       first file written will always be zero, the second one, etc.
     - The directory can *only* depend `port_name` and `image_type`. It _cannot_
       depend on values that change over time (e.g., `time_double`, `count`,
       etc.

   @param port_name         The name of the port (must be unique among all image
                            ports). This string is available in the format
                            string as `port_name`.
   @param file_name_format  The `fmt::format()`-compatible string which defines
                            the context-dependent file name to write the image
                            to.
   @param publish_period    The period at which images read from this input port
                            are written in calls to Publish().
   @param start_time        The minimum value for the context's time at which
                            images will be written in calls to Publish().
   @tparam kPixelType       The representation of the per-pixel data (see
                            PixelType). Must be one of {PixelType::kRgba8U,
                            PixelType::kDepth32F, PixelType::kLabel16I,
                            PixelType::kDepth16U, or PixelType::kGrey8U}.
   @throws std::exception   if (1) the directory encoded in the
                            `file_name_format` is not "valid" (see
                            documentation above for definition),
                            (2) `publish_period` is not positive, or
                            (3) `port_name` is used by a previous input port.
  */
  template <PixelType kPixelType>
  const InputPort<double>& DeclareImageInputPort(std::string port_name,
                                                 std::string file_name_format,
                                                 double publish_period,
                                                 double start_time);

 private:
#ifndef DRAKE_DOXYGEN_CXX
  // Friend for facilitating unit testing.
  friend class ImageWriterTester;
#endif

  // Does the work of writing image indexed by `index` to the disk.
  template <PixelType kPixelType>
  void WriteImage(const Context<double>& context, int index) const;

  // Creates a file name from the given format string and time.
  std::string MakeFileName(const std::string& format, PixelType pixel_type,
                           double time, const std::string& port_name,
                           int count) const;

  // Given the file format string (and port-specific configuration values),
  // extracts, tests, and returns the output folder information.
  // The return value will not contain a trailing slash.
  // The tests are in support of the statement that the directory path cannot
  // depend on time.
  // Examples:
  //  "a/b/c/" --> thrown exception.
  //  "a/b/c" --> "a/b"
  //  "a/{time_usec}/c" --> thrown exception.
  //  "a/{port_name}/c" --> "a/my_port"  (assuming port_name = "my_port").
  std::string DirectoryFromFormat(const std::string& format,
                                   const std::string& port_name,
                                   PixelType pixel_type) const;

  enum class FolderState {
    kValid,
    kMissing,
    kIsFile,
    kUnwritable
  };

  // Returns true if the directory path provided is valid: it exists, it's a
  // directory, and it's writable.
  static FolderState ValidateDirectory(const std::string& file_path);

  // The per-input port data.
  struct ImagePortInfo {
    ImagePortInfo(std::string format_in, PixelType pixel_type_in)
        : format(std::move(format_in)), pixel_type(pixel_type_in) {}
    const std::string format;
    const PixelType pixel_type;
    // NOTE: This is made mutable as a low-cost mechanism for incrementing
    // image writes without involving the overhead of discrete state.
    mutable int count{0};
    // TODO(SeanCurtis-TRI): For copying this system, it may be necessary to
    // also store the period and start time so that the ports in the copy can
    // be properly instantiated.
  };

  // For each input port, this stores the corresponding image data. It is an
  // invariant that port_info_.size() == num_input_ports().
  std::vector<ImagePortInfo> port_info_;

  std::unordered_map<PixelType, std::string> labels_;
  std::unordered_map<PixelType, std::string> extensions_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
