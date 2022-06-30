#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gltf_client/internal_http_service.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

/* The type of image being rendered. */
enum RenderImageType {
  kColorRgba8U = 0,    ///< The color frame.
  kLabel16I = 1,       ///< The label frame.
  kDepthDepth32F = 2,  ///< The depth frame.
};

/* TODO(svenevs): link to the render-client specification document once we
 decide where in the documentation it will live. */
/* The client which communicates with a render server. */
class RenderClient {
 public:
  /* Constructs the render engine from the given RenderEngineGltfClientParams.

   @note
     RenderEngineGltfClientParams.default_label struct member is not relavant
     for the RenderClient construction.
   @throws std::exception
     If a wrong base_url or render_endpoint is provided. */
  explicit RenderClient(const RenderEngineGltfClientParams& params);

  ~RenderClient();

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RenderClient);

  /* @name Server communication */
  //@{

  /* Uploads the scene file from `scene_path` to the render server, downloads
   the image file response, and returns the path to the image file.
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
   @param depth_range
     When `image_type` describes a depth render, this parameter must be provided
     from the DepthRenderCamera::depth_range().  May not have a value for color
     or label image renders.
   @return
     A successful download of a rendering from the server will return the path
     to the downloaded file, which will be exactly
     `{temp_directory()}/{$(basename scene_path)} + extension`, where
     `extension` will depend on what the server returns, e.g., `.png` or
     `.tiff`.
   @throws std::runtime_error
     If a rendering cannot be obtained from the server for any reason.
   @throws std::logic_error
     If `image_type` is a depth image but `depth_range` was not provided, or
     `depth_range` was provided but `image_type` is color or label. */
  std::string RenderOnServer(
      const drake::geometry::render::RenderCameraCore& camera_core,
      RenderImageType image_type, const std::string& scene_path,
      const std::optional<std::string>& mime_type = std::nullopt,
      const std::optional<drake::geometry::render::DepthRange>& depth_range =
          std::nullopt) const;

  //@}
  /* @name Server communication helpers */
  //@{

  /* Computes and returns the `sha256sum` of the specified `path`.
   @throws std::runtime_error
     If the `path` cannot be opened or the hash fails to compute.
   */
  std::string ComputeSha256(const std::string& path) const;

  /* Renames the specified file `response_data_path` to have the same name as
   `reference_path`, with a new file extension provided by `extension`.
   Helper method for RetrieveRender() which will download files as
   `{temp_directory()}/{response_data_path}` and then rename the file
   depending on the type of image that was downloaded.  Examples:

   @code{.cpp}
    const auto renamed_1 = RenameHttpServiceResponse(
        "/some/other/ABC.curl",
        "/some/input/XYZ.gltf",
        ".png");
    // renamed_1: "/some/input/XYZ.png"
    const auto renamed_2 = RenameHttpServiceResponse(
        "/a/folder/999.bin",
        "/a/folder/123.gltf",
        ".tiff");
    // renamed_2: "/a/folder/123.tiff"
   @endcode

   @param response_data_path
     The input path to change the file extension for, e.g., `"file.curl"`.
     Should be the value of HttpResponse::data_path.
   @param reference_path
     The reference path to be renaming `response_data_path` to, with a different
     file extension, e.g., `"scene.gltf"`.
   @param extension
     The new file extension, e.g., `".png"`.  Uses
     `std::filesystem::path::replace_extension()` internally.
   @return
     The path to the new file after renaming it.
   @throws std::exception
     If `input_scene` or `path` do not exist, or any errors arise from renaming
     the file. */
  std::string RenameHttpServiceResponse(const std::string& response_data_path,
                                        const std::string& reference_path,
                                        const std::string& extension) const;

  //@}

  /* @name Image loading helpers */
  //@{

  /* Loads the specified image file to a drake output buffer.

   This method only supports loading unsigned char PNG images with either three
   (RGB) or four (RGBA) channels.

   @param path
     The path to the file to try and load as a depth image.  The path returned
     by RetrieveRender() can be used directly for this parameter.
   @param color_image_out
     The already allocated drake image buffer to load `path` into.

   @throws std::runtime_error
     If the specified `path` cannot be loaded as an unsigned char RGB or RGBA
     PNG file, or the image denoted by `path` does not have the same width and
     height of the specified `color_image_out`.*/
  void LoadColorImage(
      const std::string& path,
      drake::systems::sensors::ImageRgba8U* color_image_out) const;

  /* Loads the specified image file to a drake output buffer.

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
     If the specified `path` cannot be loaded as a single channel 16-bit or
     32-bit TIFF image, or the image denoted by `path` does not have the same
     width and height of the specified `depth_image_out`. */
  void LoadDepthImage(
      const std::string& path,
      drake::systems::sensors::ImageDepth32F* depth_image_out) const;

  /* Loads the specified image file to a drake output buffer.

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
  void LoadLabelImage(
      const std::string& path,
      drake::systems::sensors::ImageLabel16I* label_image_out) const;

  //@}

  /* @name Access the default properties

   Provides access to the default values of this instance.  These values must be
   set at construction. */
  //@{

  /* Returns a RenderEngineGltfClientParams struct for RenderClient
   construction.  Note that `default_label` is not relevant for RenderClient
   class, so it's always set to std::nullopt. */
  RenderEngineGltfClientParams get_params() const {
    return RenderEngineGltfClientParams{base_url_, render_endpoint_,
                                        std::nullopt, verbose_, no_cleanup_};
  }

  /* The temporary directory used for scratch space, including but not limited
   to where downloaded images are saved.  Child classes are permitted (and
   encouraged) to utilize this directory to create any additional files needed
   to communicate with the server such as scene files to upload.  The temporary
   directory will be deleted upon destruction of this instance unless
   no_cleanup() is true. */
  const std::string& temp_directory() const { return temp_directory_; }

  /* The base url of the server. The full url to communicate with is constructed
   as `base_url()/render_endpoint()`. */
  const std::string& base_url() const { return base_url_; }

  /* The render endpoint of the server, used in RetrieveRender().
   Should **not** include a preceding slash. */
  const std::string& render_endpoint() const { return render_endpoint_; }

  /* Whether or not the client should be verbose including logging all curl
   communications. */
  bool verbose() const { return verbose_; }

  /* Whether or not the client should cleanup its temp_directory(). */
  bool no_cleanup() const { return no_cleanup_; }

  //@}

  /* (Internal use only) for testing. */
  void SetHttpService(std::unique_ptr<HttpService> service);

 private:
  friend class RenderClientTester;
  const std::string temp_directory_;
  const std::string base_url_;
  const std::string render_endpoint_;
  const bool verbose_;
  const bool no_cleanup_;
  const std::string url_;
  std::unique_ptr<HttpService> http_service_;
};

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
