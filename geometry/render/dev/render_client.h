#pragma once

#include <optional>
#include <string>

#include <curl/curl.h>

#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/* TODO(svenevs): link to the render-client specification document once we
 decide where in the documentation it will live. */
/** The client which communicates with a render server. */
class RenderClient {
 public:
  /** \name Does not allow copy, move, or assignment  */
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
   \param url
     The url of the server to communicate with, e.g., `"http://127.0.0.1"`.
   \param port
     The port to communicate with the server on, e.g., `8000`.  A value of `0`
     implies no port-level communication is needed.
   \param upload_endpoint
     The endpoint that the server expects to receive scene file uploads to,
     e.g., `"upload"`.  Do not include a preceding `/`, communications with the
     server are constructed as `{url}/{upload_endpoint}`.
   \param render_endpoint
     The endpoint that the server expects to receive render requests to, e.g.,
     `"render"`.  Do not include a preceding `/`, communications with the server
     are constructed as `{url}/{render_endpoint}`.
   \param verbose
     Whether or not the client should be verbose in logging its communications
     with the server.
   \param no_cleanup
     Whether or not the \ref temporary_directory() should be deleted upon
     destruction of this instance. */
  explicit RenderClient(const std::string& url, unsigned port,
                        const std::string& upload_endpoint,
                        const std::string& render_endpoint, bool verbose,
                        bool no_cleanup);

 protected:
  /** Copy constructor for the purpose of cloning. */
  RenderClient(const RenderClient& other);

  /** \name Server communication */
  //@{

  /** Compute and return the `sha256sum` of the specified `path`.
   \throws std::runtime_error
     If the `path` cannot be opened or the hash fails to compute.
   */
  std::string ComputeSha256(const std::string& path) const;

  /** Upload the scene to the render server.
   \param image_type   The type of scene being uploaded.
   \param scene_path   The path to the scene file to upload to the server.
   \param scene_sha256 The `sha256sum` of the file denoted by `scene_path`.
   \throws std::runtime_error
     If the file cannot be uploaded to the server successfully, or if the server
     does not respond with the same `sha256sum` of the uploaded file. */
  virtual void UploadScene(ImageType image_type, const std::string& scene_path,
                           const std::string& scene_sha256) const;

  /** Validates the specified depth range.  Helper method used in
   \ref RetrieveRender when the `image_type` is depth.  \sa DepthRange
   \param min_depth The minimum depth range.
   \param max_depth The maximum depth range.
   \throws std::logic_error
     If `min_depth` or `max_depth` are less than `0.0`, or if
     `max_depth <= min_depth`. */
  void ValidDepthRangeOrThrow(double min_depth, double max_depth) const;

  /** Download a render from the server, returning the path to the file.  Users
   should have already uploaded the scene to the server using \ref UploadScene.
   Note that the returned file path will have a `.bin` file extension as this
   method is agnostic to the type of image file being transmitted.  Users can
   call any one of the helper methods \ref LoadColorImage, \ref LoadDepthImage,
   or \ref LoadLabelImage directly with the return value of this function
   without needing to rename the file.

   \param camera_core
     The \ref RenderCameraCore of the camera being rendered.  Its
     \ref systems::sensors::CameraInfo intrinsics will be communicated to the
     server.
   \param image_type
     The type of image being rendered.
   \param scene_path
     The path to the input scene that is being rendered.
   \param scene_sha256
     The `sha256sum` of the file denoted by `scene_path`.  This is the "scene
     identifier" that the server uses to determine which file to render.
   \param min_depth
     The minimum depth range.  Required when `image_type` is depth,
     \sa ValidDepthRangeOrThrow.
   \param max_depth
     The maximum depth range.  Required when `image_type` is depth,
     \sa ValidDepthRangeOrThrow.

   \return
     A successful download of a rendering from the server will return the path
     to the downloaded file, which will be exactly `{scene_path} + ".bin"`.
     This method simply writes the data to a binary file, and is not aware of
     what kind of output image type is desired or required for the `image_type`
     being rendered.

   \throws std::runtime_error
     If a rendering cannot be obtained from the server for any reason. */
  virtual std::string RetrieveRender(const RenderCameraCore& camera_core,
                                     ImageType image_type,
                                     const std::string& scene_path,
                                     const std::string& scene_sha256,
                                     double min_depth = -1.0,
                                     double max_depth = -1.0) const;
  //@}

  /** \name Image loading helpers */
  //@{

  /** Load the specified image file to a drake output buffer.

   This method only supports loading unsigned char PNG images with either three
   (RGB) or four (RGBA) channels.

   \param path
     The path to the file to try and load as a color image.  The file extension
     does not matter, a path returned from \ref RetrieveRender can be used
     directly.
   \param color_image_out
     The already allocated drake image buffer to load `path` into.
   \param rename
     Whether or not the file should be renamed to the appropriate file extension
     `.png`.  Default: `true`.

   \return
     The path of the final file.  If `rename=false`, this is the same value as
     the input `path`.  Otherwise, the file extension will be changed to end
     with `.png`.

   \throws std::runtime_error
     If the specified `path` cannot be loaded as an RGB or RGBA PNG file, the
     image denoted by `path` does not have the same width and height of the
     specified `color_image_out`, or there was a problem renaming the file when
     `rename=true`. */
  virtual std::string LoadColorImage(
      const std::string& path,
      drake::systems::sensors::ImageRgba8U* color_image_out,
      bool rename = true) const;

  /** Load the specified image file to a drake output buffer.

   This method supports loading:

   - Single channel 16 bit or 32 bit TIFF images.  TIFF images are assumed to
     be encoded in units of meters.
   - TODO(svenevs): Single channel 16 bit unsigned short PNG images.  PNG images
     are assumed to be encoded in units of millimeters and will be converted to
     meters when copying to the output buffer.

   \param path
     The path to the file to try and load as a depth image.  The file extension
     does not matter, a path returned from \ref RetrieveRender can be used
     directly.
   \param depth_image_out
     The already allocated drake image buffer to load `path` into.
   \param rename
     Whether or not the file should be renamed to the appropriate file extension
     depending on if it was loaded as `.tiff` or `.png`.  Default: `true`.

   \return
     The path of the final file.  If `rename=false`, this is the same value as
     the input `path`.  Otherwise, the file extension will be changed to the
     type of file loaded.

   \throws std::runtime_error
     If the specified `path` cannot be loaded as a single channel TIFF or PNG,
     image, the image denoted by `path` does not have the same width and height
     of the specified `depth_image_out`, or there was a problem renaming the
     file when `rename=true`. */
  virtual std::string LoadDepthImage(
      const std::string& path,
      drake::systems::sensors::ImageDepth32F* depth_image_out,
      bool rename = true) const;

  /** Load the specified image file to a drake output buffer.

   This method only supports loading single channel unsigned short PNG images.

   \param path
     The path to the file to try and load as a label image.  The file extension
     does not matter, a path returned from \ref RetrieveRender can be used
     directly.
   \param label_image_out
     The already allocated drake image buffer to load `path` into.
   \param rename
     Whether or not the file should be renamed to the appropriate file extension
     `.png`.  Default: `true`.

   \return
     The path of the final file.  If `rename=false`, this is the same value as
     the input `path`.  Otherwise, the file extension will be changed to end
     with `.png`.

   \throws std::runtime_error
     If the specified `path` cannot be loaded as a single channel unsigned short
     PNG image, the image denoted by `path` does not have the same width and
     height of the specified `label_image_out`, or there was a problem renaming
     the file when `rename=true`. */
  virtual std::string LoadLabelImage(
      const std::string& path,
      drake::systems::sensors::ImageLabel16I* label_image_out,
      bool rename = true) const;

  /** Rename the specified file with the provided extension.  Helper method for
   \ref LoadColorImage, \ref LoadDepthImage, and \ref LoadLabelImage.

   \param path
     The input path to change the file extension for, e.g., `"file.bin"`.
   \param ext
     The new file extension, e.g., `".png"`.  Uses
     `std::filesystem::path::replace_extension` internally.
   \return
     The path to the new file after renaming it.
   \throws
     When any errors arise from renaming the file. */
  std::string RenameFileExtension(const std::string& path,
                                  const std::string& ext) const;

  //@}

  /** \name Access the default properties

   Provides access to the default values of this instance.  These values must be
   set at construction. */
  //@{

  /** The temporary directory used for scratch space, including but not limited
   to where downloaded images are saved.  Child classes are permitted (and
   encouraged) to utilize this directory to create any additional files needed
   to communicate with the server such as scene files to upload.  The temporary
   directory will be deleted upon destruction of this instance unless
   @ref no_cleanup is true. */
  const std::string& temp_directory() const { return temp_directory_; }

  /** The url of the server to communicate with. */
  const std::string& url() const { return url_; }

  /** The port of the server to communicate on.  `0` means no port. */
  unsigned port() const { return port_; }

  /** The upload endpoint of the server, used in @ref UploadScene. */
  const std::string& upload_endpoint() const { return upload_endpoint_; }

  /** The render endpoint of the server, used in @ref RetrieveRender. */
  const std::string& render_endpoint() const { return render_endpoint_; }

  /** Whether the client should be verbose. */
  bool verbose() const { return verbose_; }

  /** Whether the client should cleanup its @ref temp_directory. */
  bool no_cleanup() const { return no_cleanup_; }

  //@}

 private:
  std::string temp_directory_;
  std::string url_;
  unsigned port_;
  std::string upload_endpoint_;
  std::string render_endpoint_;
  bool verbose_;
  bool no_cleanup_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
