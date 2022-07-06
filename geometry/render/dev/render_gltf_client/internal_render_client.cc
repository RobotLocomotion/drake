#include "drake/geometry/render/dev/render_gltf_client/internal_render_client.h"

#include <atomic>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <type_traits>
#include <utility>  // std::pair
#include <vector>

#include <fmt/format.h>
#include <picosha2.h>
#include <vtkImageData.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkPNGReader.h>
#include <vtkTIFFReader.h>

#include "drake/common/filesystem.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/temp_directory.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/render_gltf_client/internal_http_service_curl.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace {

using drake::geometry::render::DepthRange;
using drake::geometry::render::RenderCameraCore;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::ImageLabel16I;
using drake::systems::sensors::ImageRgba8U;
namespace fs = drake::filesystem;

/* Adds field_name = field_data to the map, assumes data_map does **not**
 already have the key `field_name`. */
void AddField(DataFieldsMap* data_map, const std::string& field_name,
              const std::string& field_data) {
  (*data_map)[field_name] = field_data;
}

/* Template overload so that we do not have to std::to_string every member of
 the intrinsics() below. */
template <typename T>
void AddField(DataFieldsMap* data_map, const std::string& field_name,
              T field_data) {
  static_assert(
      std::is_arithmetic_v<T> && !std::is_enum_v<T>,
      "Manually convert field_data to a string, or specialize this function.");
  AddField(data_map, field_name, std::to_string(field_data));
}

// Overload required to prevent bad conversions between const char* and long.
template <>
void AddField<const char*>(DataFieldsMap* data_map,
                           const std::string& field_name,
                           const char* field_data) {
  AddField(data_map, field_name, std::string(field_data));
}

// Overload for RenderImageType, to avoid bad conversion via std::to_string.
template <>
void AddField<RenderImageType>(DataFieldsMap* data_map,
                               const std::string& field_name,
                               RenderImageType field_data) {
  if (field_data == RenderImageType::kColorRgba8U) {
    AddField(data_map, field_name, "color");
  } else if (field_data == RenderImageType::kDepthDepth32F) {
    AddField(data_map, field_name, "depth");
  } else if (field_data == RenderImageType::kLabel16I) {
    AddField(data_map, field_name, "label");
    // no cover: will only execute with a breaking change to RenderImageType.
    // LCOV_EXCL_START
  } else {
    throw std::runtime_error(fmt::format(
        "RenderClient: unsupported RenderImageType of {} requested.",
        field_data));
  }
  // LCOV_EXCL_STOP
}

/* Verifies the loaded image has the correct dimensions.  This includes
 verifying that the image is 2D (depth=1).

 @param expected_width
   The expected width of the loaded image.
 @param expected_height
   The expected height of the loaded image.
 @param image_exporter
   The already-loaded vtk image exporter.
 @param path
   The filename of the path that image_exporter has data from.  Used to populate
   the exception message.
 @throws std::runtime_error
   If the expected_width or expected_height are not the same as image_exporter,
   or if a 3D image was provided (depth > 1).
 */
void VerifyImportedImageDimensions(int expected_width, int expected_height,
                                   vtkImageExport* image_exporter,
                                   const std::string& path) {
  const int* extent = image_exporter->GetDataExtent();
  const int image_width = extent[1] - extent[0] + 1;
  const int image_height = extent[3] - extent[2] + 1;
  if (image_width != expected_width || image_height != expected_height) {
    throw std::runtime_error(fmt::format(
        "RenderClient: expected to import (width={},height={}) from the file "
        "'{}', but got (width={},height={}).",
        expected_width, expected_height, path, image_width, image_height));
  }
  /* This method should only be getting called for loading two dimensional
   images; VTK supports 3D images so we additionally check that the depth
   dimension is 1.  A TIFF image, for example, can have multiple layers. */
  const int image_depth = extent[5] - extent[4] + 1;
  if (image_depth != 1) {
    /* no cover: no tests ever use a 3D image, but the check is important to
     keep in order to guarantee the loops in LoadColorImage, as well as calls
     to image_exporter->Export are safe. */
    // LCOV_EXCL_START
    throw std::runtime_error(fmt::format(
        "RenderClient: expected two dimensional image, but loaded image from "
        "'{}' has a z dimension of {}.",
        path, image_depth));
    // LCOV_EXCL_STOP
  }
}

}  // namespace

RenderClient::RenderClient(const RenderEngineGltfClientParams& params)
    : temp_directory_{drake::temp_directory()},
      base_url_{params.base_url},
      render_endpoint_{params.render_endpoint},
      verbose_{params.verbose},
      no_cleanup_{params.no_cleanup},
      url_{params.GetUrl()},
      http_service_{std::make_unique<HttpServiceCurl>()} {}

RenderClient::~RenderClient() {
  const fs::path temp_dir{temp_directory_};
  if (!no_cleanup_) {
    try {
      fs::remove_all(temp_dir);
      // no cover: OS dependent exceptions for fs::remove_all not known.
      // LCOV_EXCL_START
    } catch (const std::exception& e) {
      drake::log()->debug("RenderClient: could not delete '{}'. {}",
                          temp_directory_, e.what());
    }
    // LCOV_EXCL_STOP
  } else if (verbose_) {
    // NOTE: this gets printed twice because of cloning, cannot be avoided.
    drake::log()->debug(
        "RenderClient: temporary directory '{}' was *NOT* deleted.",
        temp_directory_);
  }
}

std::string RenderClient::RenderOnServer(
    const RenderCameraCore& camera_core, RenderImageType image_type,
    const std::string& scene_path, const std::optional<std::string>& mime_type,
    const std::optional<DepthRange>& depth_range) const {
  // Make sure depth_range is only provided for depth images.
  if (image_type == RenderImageType::kDepthDepth32F &&
      !depth_range.has_value()) {
    throw std::logic_error(
        "RenderOnServer: depth image render requested, but no depth_range was "
        "provided.");
  } else if (depth_range.has_value() &&
             (image_type == RenderImageType::kColorRgba8U ||
              image_type == RenderImageType::kLabel16I)) {
    throw std::logic_error(
        "RenderOnServer: the depth_range parameter may only be provided when "
        "the image_type is a depth image.");
  }

  // Add the fields to the form.
  DataFieldsMap field_map;
  const std::string scene_sha256 = ComputeSha256(scene_path);
  AddField(&field_map, "scene_sha256", scene_sha256);
  AddField(&field_map, "image_type", image_type);
  const auto& intrinsics = camera_core.intrinsics();
  AddField(&field_map, "width", intrinsics.width());
  AddField(&field_map, "height", intrinsics.height());
  const auto& clipping = camera_core.clipping();
  AddField(&field_map, "near", clipping.near());
  AddField(&field_map, "far", clipping.far());
  AddField(&field_map, "focal_x", intrinsics.focal_x());
  AddField(&field_map, "focal_y", intrinsics.focal_y());
  AddField(&field_map, "fov_x", intrinsics.fov_x());
  AddField(&field_map, "fov_y", intrinsics.fov_y());
  AddField(&field_map, "center_x", intrinsics.center_x());
  AddField(&field_map, "center_y", intrinsics.center_y());
  // For depth images, an additional min_depth and max_depth are sent for the
  // depth range of the sensor (the range sensor's clipping range for valid
  // measuremeants, not the perspective clipping of the sensor's curvature).
  if (image_type == RenderImageType::kDepthDepth32F) {
    const auto& range = depth_range.value();  // has_value checked above.
    AddField(&field_map, "min_depth", range.min_depth());
    AddField(&field_map, "max_depth", range.max_depth());
  }
  AddField(&field_map, "submit", "Render");

  // Post the form and validate the results.
  auto response =
      http_service_->PostForm(temp_directory_, url_, field_map,
                              {{"scene", {scene_path, mime_type}}}, verbose_);
  if (!response.Good()) {
    /* Server may have responded with meaningful text, try and load the file
     as a string. */
    std::string server_message = "None.";
    if (response.data_path.has_value()) {
      try {
        /* See if the file is "small enough" to be json rather than an image.
         Anything larger than 8192 bytes is considered too large. */
        const auto& data_path = response.data_path.value();
        auto bin_size = fs::file_size(data_path);
        if (bin_size > 0 && bin_size < 8192) {
          std::ifstream bin_in(data_path, std::ios::binary);
          if (bin_in.is_open()) {
            std::stringstream buff;
            buff << bin_in.rdbuf();
            server_message = buff.str();
          }
        }
        // no cover: no known way to generate this error path.
        // LCOV_EXCL_START
      } catch (...) {
        server_message = "None.";
      }
      // LCOV_EXCL_STOP
    }
    throw std::runtime_error(fmt::format(
        R"(
        ERROR doing POST:  /{}
          Server Base URL: {}
          Service Message: {}
          HTTP Code:       {}
          Server Message:  {}
        )",
        render_endpoint_, base_url_,
        (response.service_error_message.has_value()
             ? response.service_error_message.value()
             : "None."),
        response.http_code, server_message));
  }

  // If the server did not respond with a file, there is nothing to load.
  if (!response.data_path.has_value()) {
    throw std::runtime_error(fmt::format(
        "ERROR with POST /{} response from server, base_url={}, HTTP code={}: "
        "the server was supposed to respond with a file but did not.",
        render_endpoint_, base_url_, response.http_code));
  }
  const std::string bin_out_path = response.data_path.value();
  if (!fs::is_regular_file(bin_out_path)) {
    throw std::runtime_error(fmt::format(
        "ERROR with POST /{} response from service, base_url={}, HTTP code={}: "
        "the service responded with a file path '{}' but the file does not "
        "exist.",
        render_endpoint_, base_url_, response.http_code, bin_out_path));
  }

  /* At this point we have a seemingly valid return file from the server, see
   if it can be loaded as one of the supported image types.  We do not check for
   the validity (e.g., underlying type or number of channels), the Load*Image
   methods are responsible for this.  However, this method will rename the file
   from e.g., "XYZ.curl" to the correct image file extension for better
   housekeeping in the temp_directory.

   If the server did not return one of the kinds of files that are supported,
   error out now.

   NOTE: do not rely on or trust the server to (correctly) report a valid mime
   type for the sent image.  VTK's image readers 'CanReadFile' methods check
   if the file *content* can actually be loaded (regardless of extension). */
  std::string image_types_tried = "";  // Build up for error message at end.

  vtkNew<vtkPNGReader> png_reader;
  if (png_reader->CanReadFile(bin_out_path.c_str())) {
    return RenameHttpServiceResponse(bin_out_path, scene_path, ".png");
  }
  image_types_tried += "PNG";

  vtkNew<vtkTIFFReader> tiff_reader;
  if (tiff_reader->CanReadFile(bin_out_path.c_str())) {
    return RenameHttpServiceResponse(bin_out_path, scene_path, ".tiff");
  }
  image_types_tried += ", TIFF";

  throw std::runtime_error(fmt::format(
      "RenderClient: while trying to render the scene '{}' with a sha256 hash "
      "of '{}', the file returned by the server saved in '{}' is not "
      "understood as an image type that is supported.  Image types attempted "
      "loading as: {}.",
      scene_path, scene_sha256, bin_out_path, image_types_tried));
}

std::string RenderClient::ComputeSha256(const std::string& path) const {
  if (!fs::is_regular_file(path)) {
    throw std::runtime_error(
        fmt::format("ComputeSha256: input file '{}' does not exist.", path));
  }
  try {
    std::ifstream f_in(path, std::ios::binary);
    if (!f_in.good()) {
      throw std::runtime_error(fmt::format("cannot open file '{}'.", path));
    }
    std::vector<unsigned char> hash(picosha2::k_digest_size);
    picosha2::hash256(f_in, hash.begin(), hash.end());
    f_in.close();
    return picosha2::bytes_to_hex_string(hash.begin(), hash.end());
  } catch (const std::exception& e) {
    throw std::runtime_error("ComputeSha256: unable to compute hash: " +
                             std::string(e.what()));
  }
}

std::string RenderClient::RenameHttpServiceResponse(
    const std::string& response_data_path, const std::string& reference_path,
    const std::string& extension) const {
  const fs::path origin{response_data_path};
  if (!fs::is_regular_file(origin)) {
    throw std::runtime_error(
        fmt::format("RenderClient: cannot rename '{}', file does not exist.",
                    response_data_path));
  }

  // Require that input path to match is valid.
  if (!fs::exists(reference_path)) {
    throw std::runtime_error(
        fmt::format("RenderClient: cannot rename '{0}' to '{1}' with extension "
                    "'{2}': '{1}' "
                    "does not exist.",
                    response_data_path, reference_path, extension));
  }

  fs::path destination{reference_path};
  destination.replace_extension(fs::path{extension});
  // Do not overwrite files blindly, require a clean directory.
  if (fs::exists(destination)) {
    throw std::runtime_error(fmt::format(
        "RenderClient: refusing to rename '{}' to '{}', file already exists!",
        origin.string(), destination.string()));
  }

  // Rename and log what changed.
  fs::rename(origin, destination);
  if (verbose_) {
    drake::log()->debug("RenderClient: renamed '{}' to '{}'.", origin.string(),
                        destination.string());
  }

  return destination;
}

void RenderClient::LoadColorImage(const std::string& path,
                                  ImageRgba8U* color_image_out) const {
  DRAKE_DEMAND(color_image_out != nullptr);

  // Load the PNG file from disk if possible.
  vtkNew<vtkPNGReader> png_reader;
  if (!png_reader->CanReadFile(path.c_str())) {
    throw std::runtime_error(
        fmt::format("RenderClient: cannot load '{}' as PNG.", path));
  }
  png_reader->SetFileName(path.c_str());
  vtkNew<vtkImageExport> image_exporter;
  image_exporter->SetInputConnection(png_reader->GetOutputPort());
  image_exporter->ImageLowerLeftOff();
  png_reader->Update();  // Loads the image.

  VerifyImportedImageDimensions(color_image_out->width(),
                                color_image_out->height(), image_exporter,
                                path);

  // For color images, we support loading either RGB (3 channels) or RGBA (4).
  vtkImageData* image_data = image_exporter->GetInput();
  DRAKE_DEMAND(image_data != nullptr);
  const int channels = image_data->GetNumberOfScalarComponents();
  if (channels != 3 && channels != 4) {
    throw std::runtime_error(fmt::format(
        "RenderClient: loaded PNG image from '{}' has {} channel(s), but "
        "either 3 (RGB) or 4 (RGBA) are required for color images.",
        path, channels));
  }

  // Make sure we have a standard PNG image with uint8_t data per channel.
  /* no cover: this case is improbable and therefore not worth explicitly
   testing. If this assumption proves to be wrong in the future, we can revisit
   the decision. */
  DRAKE_THROW_UNLESS(image_data->GetScalarType() == VTK_UNSIGNED_CHAR);

  /* Copy the image to the drake buffer.  VTK's image coordinate corners (how
   the data is stored in memory are transposed.  The separate loops is a slight
   optimization, drake's buffers are RGBA so check once and do different loops
   rather than check in every iteration of the loop. */
  using data_t = typename ImageRgba8U::T;
  data_t* out_data = color_image_out->at(0, 0);
  data_t* in_data = static_cast<data_t*>(image_data->GetScalarPointer(0, 0, 0));
  constexpr auto kNumChannels = ImageRgba8U::kNumChannels;
  static_assert(  // The logic below is not valid if this ever changes.
      kNumChannels == 4, "Expected ImageRgba8U::kNumChannels to be 4.");
  const int width = color_image_out->width();
  const int height = color_image_out->height();
  if (channels == kNumChannels) {
    // Same number of channels, do a direct transpose copy.
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int out_idx = (x + y * width) * kNumChannels;
        const int in_idx = (x + (height - y - 1) * width) * kNumChannels;
        out_data[out_idx + 0] = in_data[in_idx + 0];  // Red channel.
        out_data[out_idx + 1] = in_data[in_idx + 1];  // Blue channel.
        out_data[out_idx + 2] = in_data[in_idx + 2];  // Green channel.
        out_data[out_idx + 3] = in_data[in_idx + 3];  // Alpha channel.
      }
    }
  } else {
    // Output is RGBA, input is RGB -- transpose copy with alpha pad.
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int out_idx = (x + y * width) * kNumChannels;
        const int in_idx = (x + (height - y - 1) * width) * 3;
        out_data[out_idx + 0] = in_data[in_idx + 0];  // Red channel.
        out_data[out_idx + 1] = in_data[in_idx + 1];  // Blue channel.
        out_data[out_idx + 2] = in_data[in_idx + 2];  // Green channel.
        out_data[out_idx + 3] = 255u;                 // Alpha channel.
      }
    }
  }
}

void RenderClient::LoadDepthImage(const std::string& path,
                                  ImageDepth32F* depth_image_out) const {
  DRAKE_DEMAND(depth_image_out != nullptr);

  // Load the TIFF file from disk if possible.
  vtkNew<vtkTIFFReader> tiff_reader;
  // TODO(svenevs): add support for 16U png files.
  if (!tiff_reader->CanReadFile(path.c_str())) {
    throw std::runtime_error(
        fmt::format("RenderClient: cannot load '{}' as TIFF.", path));
  }
  tiff_reader->SetFileName(path.c_str());
  vtkNew<vtkImageExport> image_exporter;
  image_exporter->SetInputConnection(tiff_reader->GetOutputPort());
  // TODO(svenevs): why is this not needed for the TIFF import?
  // image_exporter->ImageLowerLeftOff();
  tiff_reader->Update();

  VerifyImportedImageDimensions(depth_image_out->width(),
                                depth_image_out->height(), image_exporter,
                                path);

  // For depth images, we support loading single channel images only.
  vtkImageData* image_data = image_exporter->GetInput();
  DRAKE_DEMAND(image_data != nullptr);
  const int channels = image_data->GetNumberOfScalarComponents();
  if (channels != 1) {
    throw std::runtime_error(fmt::format(
        "RenderClient: loaded TIFF image from '{}' has {} channels, but only 1 "
        "is allowed for depth images.",
        path, channels));
  }

  /* Make sure we can copy directly using VTK before doing so.  Even if a 16 bit
   TIFF image was sent, VTK will load it as 32 bit float data. */
  /* no cover: this case is improbable and therefore not worth explicitly
   testing. If this assumption proves to be wrong in the future, we can revisit
   the decision. */
  DRAKE_THROW_UNLESS(image_data->GetScalarType() == VTK_TYPE_FLOAT32);

  image_exporter->Export(depth_image_out->at(0, 0));
}

void RenderClient::LoadLabelImage(const std::string& path,
                                  ImageLabel16I* label_image_out) const {
  DRAKE_DEMAND(label_image_out != nullptr);

  // Load the PNG file from disk if possible.
  vtkNew<vtkPNGReader> png_reader;
  if (!png_reader->CanReadFile(path.c_str())) {
    throw std::runtime_error(
        fmt::format("RenderClient: cannot load '{}' as PNG.", path));
  }
  png_reader->SetFileName(path.c_str());
  vtkNew<vtkImageExport> image_exporter;
  image_exporter->SetInputConnection(png_reader->GetOutputPort());
  image_exporter->ImageLowerLeftOff();
  png_reader->Update();  // Loads the image.

  VerifyImportedImageDimensions(label_image_out->width(),
                                label_image_out->height(), image_exporter,
                                path);

  // For label images, we support loading 16 bit unsigned single channel images.
  vtkImageData* image_data = image_exporter->GetInput();
  DRAKE_DEMAND(image_data != nullptr);
  const int channels = image_data->GetNumberOfScalarComponents();
  if (channels != 1) {
    throw std::runtime_error(fmt::format(
        "RenderClient: loaded PNG image from '{}' has {} channels, but only 1 "
        "is allowed for label images.",
        path, channels));
  }

  /* no cover: this case is improbable and therefore not worth explicitly
   testing. If this assumption proves to be wrong in the future, we can revisit
   the decision. */
  DRAKE_THROW_UNLESS(image_data->GetScalarType() == VTK_TYPE_UINT16);

  /* NOTE: Officially label image is signed integers, the vtkImageExport::Export
   will reinterpret this as unsigned internally, since the loaded PNG image is
   required to be unsigned short data, negative values will not occur. */
  image_exporter->Export(label_image_out->at(0, 0));
}

void RenderClient::SetHttpService(std::unique_ptr<HttpService> service) {
  http_service_ = std::move(service);
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
