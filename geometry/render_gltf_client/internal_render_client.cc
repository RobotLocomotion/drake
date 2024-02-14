#include "drake/geometry/render_gltf_client/internal_render_client.h"

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/find_resource.h"
#include "drake/common/sha256.h"
#include "drake/common/temp_directory.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/render_gltf_client/internal_http_service_curl.h"
#include "drake/systems/sensors/image_io.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace {

namespace fs = std::filesystem;

using render::ClippingRange;
using render::DepthRange;
using render::RenderCameraCore;
using systems::sensors::CameraInfo;
using systems::sensors::ImageAny;
using systems::sensors::ImageDepth16U;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageFileFormat;
using systems::sensors::ImageIo;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

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

/* Overload required to prevent bad conversions between const char* and
 std::string. */
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
  switch (field_data) {
    case RenderImageType::kColorRgba8U: {
      AddField(data_map, field_name, "color");
      break;
    }
    case RenderImageType::kDepthDepth32F: {
      AddField(data_map, field_name, "depth");
      break;
    }
    case RenderImageType::kLabel16I: {
      AddField(data_map, field_name, "label");
      break;
    }
  }
}

/* Throws an error when the loaded image doesn't have the correct dimensions. */
template <typename Image>
void VerifyImageSize(const std::string& image_path, Image* image,
                     int expected_width, int expected_height) {
  DRAKE_DEMAND(image != nullptr);
  const int actual_width = image->width();
  const int actual_height = image->height();
  if (actual_width != expected_width || actual_height != expected_height) {
    throw std::runtime_error(fmt::format(
        "RenderClient: expected to import (width={},height={}) from the file "
        "'{}', but got (width={},height={}).",
        expected_width, expected_height, image_path, actual_width,
        actual_height));
  }
}

}  // namespace

RenderClient::RenderClient(const RenderEngineGltfClientParams& params)
    : temp_directory_{drake::temp_directory()},
      params_{params},
      http_service_{std::make_unique<HttpServiceCurl>()} {}

RenderClient::~RenderClient() {
  const fs::path temp_dir{temp_directory_};
  if (params_.cleanup) {
    try {
      fs::remove_all(temp_dir);
      // no cover: OS dependent exceptions for fs::remove_all not known.
    } catch (const std::exception& e) {
      // Note: Catching an exception is generally verboten.  However, since
      // exceptions can't be thrown in a destructor, doing so is allowed here.
      log()->debug("RenderClient: could not delete '{}'. {}", temp_directory_,
                   e.what());
    }
  } else if (params_.verbose) {
    // NOTE: This gets printed twice because of cloning, cannot be avoided.
    log()->debug("RenderClient: temporary directory '{}' was *NOT* deleted.",
                 temp_directory_);
  }
}

std::string RenderClient::RenderOnServer(
    const RenderCameraCore& camera_core, RenderImageType image_type,
    const std::string& scene_path, const std::optional<std::string>& mime_type,
    const std::optional<DepthRange>& depth_range) const {
  // Make sure depth_range is only provided for depth images.
  const bool is_depth_type = (image_type == RenderImageType::kDepthDepth32F);
  DRAKE_THROW_UNLESS(depth_range.has_value() == is_depth_type);

  // Add the fields to the form.
  DataFieldsMap field_map;
  const std::string scene_sha256 = ComputeSha256(scene_path);
  AddField(&field_map, "scene_sha256", scene_sha256);
  AddField(&field_map, "image_type", image_type);
  const CameraInfo& intrinsics = camera_core.intrinsics();
  AddField(&field_map, "width", intrinsics.width());
  AddField(&field_map, "height", intrinsics.height());
  const ClippingRange& clipping = camera_core.clipping();
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
  // measurements, not the perspective clipping of the sensor's curvature).
  if (is_depth_type) {
    const DepthRange& range = depth_range.value();  // has_value checked above.
    AddField(&field_map, "min_depth", range.min_depth());
    AddField(&field_map, "max_depth", range.max_depth());
  }
  AddField(&field_map, "submit", "Render");

  const std::string url = params_.GetUrl();
  // Post the form and validate the results.
  const HttpResponse response = http_service_->PostForm(
      temp_directory_, url, field_map, {{"scene", {scene_path, mime_type}}},
      params_.verbose);
  if (!response.Good()) {
    /* Server may have responded with meaningful text, try and load the file
     as a string. */
    std::string server_message{"None."};
    if (response.data_path.has_value()) {
      /* See if the file is "small enough" to be json rather than an image.
        Anything larger than or equal to 8192 bytes is considered too large. */
      const std::string& data_path = response.data_path.value();
      const std::uintmax_t bin_size = fs::file_size(data_path);
      if (bin_size > 0 && bin_size < 8192) {
        if (std::optional<std::string> data = ReadFile(data_path)) {
          server_message = std::move(*data);
        }
      }
    }
    const std::string service_error_message =
        response.service_error_message.value_or("None.");
    throw std::runtime_error(fmt::format(
        R"""(RenderClient: error from POST:
  URL:             {}
  Service Message: {}
  HTTP Code:       {}
  Server Message:  {})""",
        url, service_error_message, response.http_code, server_message));
  }

  // If the server did not respond with a file, there is nothing to load.
  if (!response.data_path.has_value()) {
    throw std::runtime_error(fmt::format(
        "RenderClient: error from POST to {}, HTTP code={}: the server was "
        "supposed to respond with a file but did not.",
        url, response.http_code));
  }
  const std::string bin_out_path = response.data_path.value();
  DRAKE_DEMAND(fs::is_regular_file(bin_out_path));

  /* At this point we have a seemingly valid return file from the server. See
   if it can be loaded as one of the supported image types.  We do not check for
   the validity (e.g., underlying type or number of channels), the Load*Image
   methods are responsible for this.  However, this method will rename the file
   from e.g., "XYZ.curl" to the correct image file extension for better
   housekeeping in the temp_directory.

   If the server did not return one of the supported formats, error out now. */
  const std::optional<ImageIo::Metadata> header =
      ImageIo{}.LoadMetadata(bin_out_path);
  if (header.has_value()) {
    if (header->format == ImageFileFormat::kPng) {
      return RenameHttpServiceResponse(bin_out_path, scene_path, ".png");
    }
    if (header->format == ImageFileFormat::kTiff) {
      return RenameHttpServiceResponse(bin_out_path, scene_path, ".tiff");
    }
  }
  throw std::runtime_error(fmt::format(
      "RenderClient: while trying to render the scene '{}' with a sha256 hash "
      "of '{}', the file returned by the server saved in '{}' is not "
      "understood as an image type that is supported, i.e., PNG or TIFF.",
      scene_path, scene_sha256, bin_out_path));
}

std::string RenderClient::ComputeSha256(const std::string& path) {
  std::ifstream f_in(path, std::ios::binary);
  if (!f_in.good()) {
    throw std::runtime_error(
        fmt::format("RenderClient: cannot open file '{}'.", path));
  }
  return Sha256::Checksum(&f_in).to_string();
}

std::string RenderClient::RenameHttpServiceResponse(
    const std::string& response_data_path, const std::string& reference_path,
    const std::string& extension) {
  const fs::path origin{response_data_path};
  fs::path destination{reference_path};
  destination.replace_extension(fs::path{extension});

  // Do not overwrite files blindly, require a clean directory.
  if (fs::exists(destination)) {
    throw std::runtime_error(fmt::format(
        "RenderClient: refusing to rename '{}' to '{}', file already exists!",
        origin.string(), destination.string()));
  }

  fs::rename(origin, destination);

  return destination;
}

void RenderClient::LoadColorImage(const std::string& path,
                                  ImageRgba8U* color_image_out) {
  DRAKE_DEMAND(color_image_out != nullptr);
  const int expected_width = color_image_out->width();
  const int expected_height = color_image_out->height();
  ImageIo{}.Load(path, ImageFileFormat::kPng, color_image_out);
  VerifyImageSize(path, color_image_out, expected_width, expected_height);
}

void RenderClient::LoadDepthImage(const std::string& path,
                                  ImageDepth32F* depth_image_out) {
  DRAKE_DEMAND(depth_image_out != nullptr);
  const int expected_width = depth_image_out->width();
  const int expected_height = depth_image_out->height();
  ImageAny image_any = ImageIo{}.Load(path);
  std::visit(
      [depth_image_out]<typename SomeImage>(SomeImage& image) {
        if constexpr (std::is_same_v<SomeImage, ImageDepth16U>) {
          ConvertDepth16UTo32F(image, depth_image_out);
        } else if constexpr (std::is_same_v<SomeImage, ImageDepth32F>) {
          *depth_image_out = std::move(image);
        } else {
          throw std::runtime_error(
              "RenderClient: unsupported depth pixel type");
        }
      },
      image_any);
  VerifyImageSize(path, depth_image_out, expected_width, expected_height);
}

void RenderClient::SetHttpService(std::unique_ptr<HttpService> service) {
  http_service_ = std::move(service);
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
