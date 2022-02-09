#pragma once

#include <optional>
#include <string>

#include "drake/geometry/render/render_camera.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render {

/** The type of image being rendered. */
enum RenderImageType {
  kColorRgba8U = 0,    ///< The color frame.
  kLabel16I = 1,       ///< The label frame.
  kDepthDepth32F = 2,  ///< The depth frame.
};

/* TODO(svenevs): link to the render-client specification document once we
 decide where in the documentation it will live. */
/** The client which communicates with a render server. */
class RenderClient {
 public:
  /** @name Does not allow copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy constructor operator is actually protected to serve as the
  // basis for implementing the DoClone() method.
  RenderClient(const RenderClient&) = delete;
#endif
  RenderClient& operator=(const RenderClient&) = delete;
  RenderClient(RenderClient&&) = delete;
  RenderClient& operator=(RenderClient&&) = delete;
  virtual ~RenderClient();
  //@}}

  /** Constructs the render engine from the given parameters.
   @param url
     The url of the server to communicate with, e.g., `"http://127.0.0.1"`.
   @param port
     The port to communicate with the server on, e.g., `8000`.  A value of less
     than or equal to `0` implies no port-level communication is needed.
   @param render_endpoint
     The endpoint that the server expects to receive render requests to, e.g.,
     `"render"`.  Do not include a preceding `/`, communications with the server
     are constructed as `{url}/{render_endpoint}`.
   @param verbose
     Whether or not the client should be verbose in logging its communications
     with the server.
   @param no_cleanup
     Whether or not the temp_directory() should be deleted upon destruction of
     this instance. */
  explicit RenderClient(const std::string& url, int32_t port,
                        const std::string& render_endpoint, bool verbose,
                        bool no_cleanup);

 protected:
  /** Copy constructor for the purpose of cloning. */
  RenderClient(const RenderClient& other);

  /** @name Server communication */
  //@{

  /** Upload the scene file from `scene_path` to the render server, download
   the image file response, and return the path to the image file.
   The returned file path may be used directly with any one of the helper
   methods LoadColorImage(), LoadDepthImage(), or LoadLabelImage().  The file
   path returned will be in temp_directory(), users do not need to delete the
   file manually after they are finished.

   @sa no_cleanup()
   @param camera_core
     The RenderCameraCore of the camera being rendered.  Its
     RenderCameraCore::intrinsics() will be communicated to the server.
   @param image_type
     The type of image being rendered.
   @param scene_path
     The path to the input scene that is being rendered.  The client will use
     the basename of the file path to construct the final output file, stored
     in its temp_directory().  For example, `/some/path/scene.gltf` would be
     saved in `{temp_directory()}/scene.gltf.png` (if the server returned a PNG
     file), regardless of whether `/some/path` is in temp_directory().  Users
     are strongly encouraged to store their scene files in temp_directory().
   @param mime_type
     The mime type to set for the scene file being uploaded as a file.  If not
     provided, no mime type will be sent to the server.  No validity checks on
     the value of the provided mime type are performed.
   @param min_depth
     The minimum depth range.  Required when `image_type` is depth.  See also:
     ValidDepthRangeOrThrow().
   @param max_depth
     The maximum depth range.  Required when `image_type` is depth.  See also:
     ValidDepthRangeOrThrow().
   @return
     A successful download of a rendering from the server will return the path
     to the downloaded file, which will be exactly
     `{temp_directory()}/{$(basename scene_path)} + extension`, where
     `extension` will depend on what the server returns, e.g., `.png` or
     `.tiff`.
   @throws std::runtime_error
     If a rendering cannot be obtained from the server for any reason, including
     invalid parameters supplied to this method such as not including
     `min_depth` and/or `max_depth` when `image_type` is depth. */
  virtual std::string RenderOnServer(
      const RenderCameraCore& camera_core, RenderImageType image_type,
      const std::string& scene_path,
      const std::optional<std::string>& mime_type, double min_depth = -1.0,
      double max_depth = -1.0) const;

  //@}
  /** @name Server communication helpers */
  //@{

  /** Compute and return the `sha256sum` of the specified `path`.
   @throws std::runtime_error
     If the `path` cannot be opened or the hash fails to compute.
   */
  std::string ComputeSha256(const std::string& path) const;

  /** Validates the specified depth range.  Helper method used in
   RetrieveRender() when the `image_type` is depth.

   @sa DepthRange
   @param min_depth The minimum depth range.
   @param max_depth The maximum depth range.
   @throws std::logic_error
     If `min_depth` or `max_depth` are less than `0.0`, or if
     `max_depth <= min_depth`. */
  void ValidDepthRangeOrThrow(double min_depth, double max_depth) const;

  /** Rename the specified file with the provided extension.  Helper method for
   RetrieveRender() which will download files as
   `{temp_directory()}/{scene_path}.bin` and then rename the file depending on
   the type of image that was downloaded.

   @param path
     The input path to change the file extension for, e.g., `"file.bin"`.
   @param ext
     The new file extension, e.g., `".png"`.  Uses
     `std::filesystem::path::replace_extension()` internally.
   @return
     The path to the new file after renaming it.
   @throws std::exception
     When any errors arise from renaming the file. */
  std::string RenameFileExtension(const std::string& path,
                                  const std::string& ext) const;

  //@}

  /** @name Image loading helpers */
  //@{

  /** Load the specified image file to a drake output buffer.

   This method only supports loading unsigned char PNG images with either three
   (RGB) or four (RGBA) channels.

   @param path
     The path to the file to try and load as a depth image.  The path returned
     by RetrieveRender() can be used directly for this parameter.
   @param color_image_out
     The already allocated drake image buffer to load `path` into.

   @throws std::runtime_error
     If the specified `path` cannot be loaded as an RGB or RGBA PNG file, or the
     image denoted by `path` does not have the same width and height of the
     specified `color_image_out`.*/
  virtual void LoadColorImage(
      const std::string& path,
      drake::systems::sensors::ImageRgba8U* color_image_out) const;

  /** Load the specified image file to a drake output buffer.

   This method supports loading:

   - Single channel 16 bit or 32 bit TIFF images.  TIFF images are assumed to
     be encoded in units of meters.
   - TODO(svenevs): Single channel 16 bit unsigned short PNG images.  PNG images
     are assumed to be encoded in units of millimeters and will be converted to
     meters when copying to the output buffer.

   @param path
     The path to the file to try and load as a depth image.  The path returned
     by RetrieveRender() can be used directly for this parameter.
   @param depth_image_out
     The already allocated drake image buffer to load `path` into.
   @throws std::runtime_error
     If the specified `path` cannot be loaded as a single channel TIFF or PNG,
     image, or the image denoted by `path` does not have the same width and
     height of the specified `depth_image_out`. */
  virtual void LoadDepthImage(
      const std::string& path,
      drake::systems::sensors::ImageDepth32F* depth_image_out) const;

  /** Load the specified image file to a drake output buffer.

   This method only supports loading single channel unsigned short PNG images.

   @param path
     The path to the file to try and load as a label image.  The path returned
     by RetrieveRender() can be used directly for this parameter.
   @param label_image_out
     The already allocated drake image buffer to load `path` into.
   @throws std::runtime_error
     If the specified `path` cannot be loaded as a single channel unsigned short
     PNG image, or the image denoted by `path` does not have the same width and
     height of the specified `label_image_out`. */
  virtual void LoadLabelImage(
      const std::string& path,
      drake::systems::sensors::ImageLabel16I* label_image_out) const;

  //@}

  /** @name Access the default properties

   Provides access to the default values of this instance.  These values must be
   set at construction. */
  //@{

  /** The temporary directory used for scratch space, including but not limited
   to where downloaded images are saved.  Child classes are permitted (and
   encouraged) to utilize this directory to create any additional files needed
   to communicate with the server such as scene files to upload.  The temporary
   directory will be deleted upon destruction of this instance unless
   no_cleanup() is true. */
  const std::string& temp_directory() const { return temp_directory_; }

  /** The url of the server to communicate with. */
  const std::string& url() const { return url_; }

  /** The port of the server to communicate on.  A value of less than or equal
   `0` means no port level communication is required. */
  int32_t port() const { return port_; }

  /** The render endpoint of the server, used in RetrieveRender().
   Should **not** include a preceding slash. */
  const std::string& render_endpoint() const { return render_endpoint_; }

  /** Whether or not the client should be verbose including logging all curl
   communications. */
  bool verbose() const { return verbose_; }

  /** Whether or not the client should cleanup its temp_directory(). */
  bool no_cleanup() const { return no_cleanup_; }

  //@}

 private:
  std::string temp_directory_;
  std::string url_;
  int32_t port_;
  std::string render_endpoint_;
  bool verbose_;
  bool no_cleanup_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
