#include "drake/geometry/render/dev/render_client.h"

#include <atomic>
#include <string>
#include <vector>

#include <curl/curl.h>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <picosha2.h>
#include <vtkImageData.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkPNGReader.h>
#include <vtkTIFFReader.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"

namespace drake {
namespace geometry {
namespace render {

namespace {

using json = nlohmann::json;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::ImageLabel16I;
using drake::systems::sensors::ImageRgba8U;

// Render client constructor increases, destructor decreases num_clients.
std::atomic<int> num_clients{0};
// is_initialized should only be modified in static_curl_{init,cleanup}.
std::atomic<bool> is_initialized{false};

void static_curl_init() {
  if (!is_initialized) {
    curl_global_init(CURL_GLOBAL_ALL);
    is_initialized = true;
  }
}

void static_curl_cleanup() {
  if (num_clients == 0) {
    curl_global_cleanup();
    is_initialized = false;
  }
}

// Write callback for libcurl, assumes `userp` points to an std::string.
// See: https://curl.se/libcurl/c/libcurl-tutorial.html
size_t write_string_data(void* buffer, size_t size, size_t nmemb, void* userp) {
  const size_t data_size = size * nmemb;
  std::string* s = static_cast<std::string*>(userp);
  s->append(static_cast<char*>(buffer), data_size);
  return data_size;
}

// header_callback

size_t write_file_data(void* buffer, size_t size, size_t nmemb, void* userp) {
  const size_t data_size = size * nmemb;
  std::ofstream* f = static_cast<std::ofstream*>(userp);
  f->write(static_cast<char*>(buffer), data_size);
  return data_size;
}

// Fill out <input type="text" name="image_type"> for an already created curl
// form with the specified image_type.
void AddImageTypeFieldToForm(curl_mime* form, RenderImageType image_type) {
  DRAKE_DEMAND(form != nullptr);
  // NOTE: field gets freed with curl_easy_cleanup(curl).
  curl_mimepart* field{nullptr};
  field = curl_mime_addpart(form);
  curl_mime_name(field, "image_type");
  std::string image_type_str;
  if (image_type == RenderImageType::kColorRgba8U) {
    image_type_str = "color";
  } else if (image_type == RenderImageType::kDepthDepth32F) {
    image_type_str = "depth";
  } else if (image_type == RenderImageType::kLabel16I) {
    image_type_str = "label";
  } else {
    throw std::runtime_error(fmt::format(
        "RenderClient: unsupported RenderImageType of {} requested.",
        image_type));
  }
  curl_mime_data(field, image_type_str.c_str(), CURL_ZERO_TERMINATED);
}

/** Verify the loaded image has the correct dimensions.

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
   If the expected_width or expected_height are not the same as image_exporter.
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
   dimension is 1. */
  const int image_depth = extent[5] - extent[4] + 1;
  if (image_depth != 1) {
    throw std::runtime_error(fmt::format(
        "RenderClient: expected two dimensional image, but loaded image from "
        "'{}' has a z dimension of {}.",
        path, image_depth));
  }
}

}  // namespace

RenderClient::RenderClient(const std::string& url, unsigned port,
                           const std::string& upload_endpoint,
                           const std::string& render_endpoint, bool verbose,
                           bool no_cleanup)
    : temp_directory_{drake::temp_directory()},
      url_{url},
      port_{port},
      upload_endpoint_{upload_endpoint},
      render_endpoint_{render_endpoint},
      verbose_{verbose},
      no_cleanup_{no_cleanup} {
  static_curl_init();
  ++num_clients;
}

RenderClient::RenderClient(const RenderClient& other)
    : temp_directory_{other.temp_directory_},
      url_{other.url_},
      port_{other.port_},
      upload_endpoint_{other.upload_endpoint_},
      render_endpoint_{other.render_endpoint_},
      verbose_{other.verbose_},
      no_cleanup_{other.no_cleanup_} {
  // Clone + no_cleanup may result in this being deleted.
  const drake::filesystem::path temp_dir{temp_directory_};
  if (!drake::filesystem::is_directory(temp_dir)) {
    temp_directory_ = drake::temp_directory();
  }
  static_curl_init();
  ++num_clients;
}

RenderClient::~RenderClient() {
  const drake::filesystem::path temp_dir{temp_directory_};
  if (!no_cleanup_) {
    if (drake::filesystem::is_directory(temp_dir)) {
      try {
        drake::filesystem::remove_all(temp_dir);
      } catch (const std::exception& e) {
        // TODO(svenevs): what is the right thing to do if we cannot delete?
        std::cerr << "WARNING: could not delete '" << temp_directory_ << "'. "
                  << e.what() << '\n';
      }
    }
  } else {
    // TODO(svenevs): make this `else if (verbose_)` instead?  This gets printed
    // twice because of cloning.
    std::cout << "RenderClient temporary directory '" << temp_directory_
              << "' was *NOT* deleted.\n";
  }
  --num_clients;
  static_curl_cleanup();
}

void RenderClient::UploadScene(
    RenderImageType image_type, const std::string& scene_path,
    const std::string& scene_sha256,
    const std::optional<std::string>& mime_type) const {
  CURL* curl{nullptr};
  CURLcode result;
  curl = curl_easy_init();
  DRAKE_DEMAND(curl != nullptr);

  curl_easy_setopt(curl, CURLOPT_VERBOSE, static_cast<int64_t>(verbose_));

  // Setup the POST url.
  const std::string upload_url = url_ + "/" + upload_endpoint_;
  curl_easy_setopt(curl, CURLOPT_URL, upload_url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  if (port_ > 0) curl_easy_setopt(curl, CURLOPT_PORT, port_);

  // Create and fill out a <form> to POST.
  curl_mime* form{nullptr};
  curl_mimepart* field{nullptr};
  struct curl_slist* headerlist{nullptr};
  form = curl_mime_init(curl);

  // Fill out <input type="file" name="data">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "data");
  curl_mime_filedata(field, scene_path.c_str());
  if (mime_type.has_value()) curl_mime_type(field, mime_type.value().c_str());

  // Fill out <input type="text" name="image_type">
  AddImageTypeFieldToForm(form, image_type);

  // Fill out <input type="submit" value="Upload" name="submit">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "submit");
  curl_mime_data(field, "Upload", CURL_ZERO_TERMINATED);

  curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);

  // Disable 100-Continue.  See:
  // http://www.iandennismiller.com/posts/curl-http1-1-100-continue-and-multipartform-data-post.html
  headerlist = curl_slist_append(headerlist, "Expect:");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);

  // Receive json response from the server.
  std::string post_response_text;
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &post_response_text);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &write_string_data);

  // Perform the POST.
  result = curl_easy_perform(curl);

  int64_t http_code{0};
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  const bool http_code_ok = http_code >= 200 && http_code < 400;
  if (result != CURLE_OK || !http_code_ok) {
    std::string server_message{""};
    if (post_response_text.length() > 0) {
      server_message = post_response_text;
    } else {
      server_message = "None.";
    }
    throw std::runtime_error(fmt::format(
        R"(
        ERROR doing POST: /{}
          Server URL:     {}
          cURL Message:   {}
          HTTP Code:      {}
          Server Message: {}
        )",
        upload_endpoint_, url_ + (port_ ? fmt::format(":{}", port_) : ""),
        curl_easy_strerror(result), http_code, server_message));
  }
  curl_slist_free_all(headerlist);
  curl_easy_cleanup(curl);

  // Validate the server response.
  std::string server_sha256;
  try {
    const json post_response = json::parse(post_response_text);
    server_sha256 = post_response["sha256"];
  } catch (const std::exception& e) {
    throw std::runtime_error(
        "Error obtaining `sha256` key from anticpated json server response `" +
        post_response_text + "`. " + e.what());
  }
  if (server_sha256 != scene_sha256) {
    throw std::runtime_error(fmt::format(
        "Error uploading scene '{}', expected sha256='{}' but the server "
        "responded with '{}'.",
        scene_path, scene_sha256, server_sha256));
  }
}

std::string RenderClient::RetrieveRender(const RenderCameraCore& camera_core,
                                         RenderImageType image_type,
                                         const std::string& scene_path,
                                         const std::string& scene_sha256,
                                         double min_depth,
                                         double max_depth) const {
  if (image_type == RenderImageType::kDepthDepth32F)
    ValidDepthRangeOrThrow(min_depth, max_depth);

  CURL* curl{nullptr};
  CURLcode result;
  curl = curl_easy_init();
  DRAKE_DEMAND(curl != nullptr);

  curl_easy_setopt(curl, CURLOPT_VERBOSE, static_cast<int64_t>(verbose_));

  // Setup the POST url.
  const std::string post_url = url_ + "/" + render_endpoint_;
  curl_easy_setopt(curl, CURLOPT_URL, post_url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  if (port_ > 0) curl_easy_setopt(curl, CURLOPT_PORT, port_);

  // Create and fill out a <form> to POST.
  curl_mime* form{nullptr};
  curl_mimepart* field{nullptr};
  struct curl_slist* headerlist{nullptr};
  form = curl_mime_init(curl);

  // Fill out <input type="text" name="id">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "id");
  curl_mime_data(field, scene_sha256.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="text" name="image_type">
  AddImageTypeFieldToForm(form, image_type);

  // Get the width / height of the image we are rendering.
  const auto& intrinsics = camera_core.intrinsics();
  const int cam_width = intrinsics.width();
  const int cam_height = intrinsics.height();
  const std::string width = fmt::format("{}", cam_width);
  const std::string height = fmt::format("{}", cam_height);

  // Fill out <input type="number" name="width">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "width");
  curl_mime_data(field, width.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="height">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "height");
  curl_mime_data(field, height.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="near">
  const auto& clipping = camera_core.clipping();
  const std::string near = fmt::format("{}", clipping.near());
  field = curl_mime_addpart(form);
  curl_mime_name(field, "near");
  curl_mime_data(field, near.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="far">
  const std::string far = fmt::format("{}", clipping.far());
  field = curl_mime_addpart(form);
  curl_mime_name(field, "far");
  curl_mime_data(field, far.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="focal_x">
  const std::string focal_x = fmt::format("{}", intrinsics.focal_x());
  field = curl_mime_addpart(form);
  curl_mime_name(field, "focal_x");
  curl_mime_data(field, focal_x.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="focal_y">
  const std::string focal_y = fmt::format("{}", intrinsics.focal_y());
  field = curl_mime_addpart(form);
  curl_mime_name(field, "focal_y");
  curl_mime_data(field, focal_y.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="fov_x">
  const std::string fov_x = fmt::format("{}", intrinsics.fov_x());
  field = curl_mime_addpart(form);
  curl_mime_name(field, "fov_x");
  curl_mime_data(field, fov_x.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="fov_y">
  const std::string fov_y = fmt::format("{}", intrinsics.fov_y());
  field = curl_mime_addpart(form);
  curl_mime_name(field, "fov_y");
  curl_mime_data(field, fov_y.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="center_x">
  const std::string center_x = fmt::format("{}", intrinsics.center_x());
  field = curl_mime_addpart(form);
  curl_mime_name(field, "center_x");
  curl_mime_data(field, center_x.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="center_y">
  const std::string center_y = fmt::format("{}", intrinsics.center_y());
  field = curl_mime_addpart(form);
  curl_mime_name(field, "center_y");
  curl_mime_data(field, center_y.c_str(), CURL_ZERO_TERMINATED);

  // For depth images, an additional min_depth and max_depth are sent for the
  // depth range of the sensor (the range sensor's clipping range for valid
  // measuremeants, not the perspective clipping of the sensor's curvature).
  if (image_type == RenderImageType::kDepthDepth32F) {
    // Fill out <input type="number" name="min_depth">
    const std::string min_depth_str = fmt::format("{}", min_depth);
    field = curl_mime_addpart(form);
    curl_mime_name(field, "min_depth");
    curl_mime_data(field, min_depth_str.c_str(), CURL_ZERO_TERMINATED);

    // Fill out <input type="number" name="max_depth">
    const std::string max_depth_str = fmt::format("{}", max_depth);
    field = curl_mime_addpart(form);
    curl_mime_name(field, "max_depth");
    curl_mime_data(field, max_depth_str.c_str(), CURL_ZERO_TERMINATED);
  }

  // Fill out <input type="submit" name="submit" value="Render">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "submit");
  curl_mime_data(field, "Render", CURL_ZERO_TERMINATED);

  curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);

  // Disable 100-Continue.  See:
  // http://www.iandennismiller.com/posts/curl-http1-1-100-continue-and-multipartform-data-post.html
  headerlist = curl_slist_append(headerlist, "Expect:");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);

  /* Have curl write directly to a file.  The provided scene_path does not
   necessarily reside in temp_directory_, construct a path that is safe for this
   RenderClient to write to. */
  const drake::filesystem::path fs_scene_path{scene_path};
  const drake::filesystem::path temp_bin_out =
      drake::filesystem::path(temp_directory_) /
      (drake::filesystem::path{fs_scene_path.filename().string() + ".bin"});
  // Start with a clean file each time.
  if (drake::filesystem::exists(temp_bin_out)) {
    try {
      if (!drake::filesystem::remove(temp_bin_out)) {
        throw std::runtime_error("unable to remove file.");
      }
    } catch (const std::exception& e) {
      throw std::runtime_error(fmt::format(
          "RenderClient: error reusing '{}' to retrieve a new render, consider "
          "calling RetrieveRender with unique scene_path arguments to receive "
          "a unique temporary file each time: {}",
          temp_bin_out.string(), e.what()));
    }
  }
  // Open the file for writing, pass it off to curl.
  const std::string bin_out_path{temp_bin_out.string()};
  std::ofstream bin_out(bin_out_path, std::ios::binary);
  if (!bin_out.good()) {
    const auto state = bin_out.rdstate();
    std::string why{""};
    if (state == std::ios_base::badbit) {
      why = "badbit was set.";
    } else if (state == std::ios_base::failbit) {
      why = "failbit was set.";
    } else {
      why = "unknown error.";
    }

    throw std::runtime_error(fmt::format(
        "RenderClient: unable to open '{}' for writing results from server to: "
        "{}",
        bin_out_path, why));
  }
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &write_file_data);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &bin_out);

  // Perform the POST.
  result = curl_easy_perform(curl);

  // Write callback is complete, close the file.
  bin_out.close();

  int64_t http_code{0};
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  const bool http_code_ok = http_code >= 200 && http_code < 400;
  if (result != CURLE_OK || !http_code_ok) {
    // Server may have responded with meaningful text, try and load the file
    // as a string.
    std::string server_message = "Server Message: ";
    try {
      // See if the file is "small enough" to be json rather than an image.
      // Anything larger than 4096 bytes is considered too large.
      auto bin_size = drake::filesystem::file_size(bin_out_path);
      bool added_message = false;
      if (bin_size > 0 && bin_size < 4096) {
        std::streampos bin_in_size{0};
        std::ifstream bin_in(bin_out_path, std::ios::binary);
        if (bin_in.is_open()) {
          std::stringstream buff;
          buff << bin_in.rdbuf();
          server_message += buff.str();
          added_message = true;
        }
      }
      if (!added_message) {
        server_message += "None.";
      }
    } catch (...) {
      server_message += "None.";
    }
    throw std::runtime_error(fmt::format(
        R"(
        ERROR doing POST: /{}
          Server URL:     {}
          cURL Message:   {}
          HTTP Code:      {}
          {}
        )",
        render_endpoint_, url_ + (port_ ? fmt::format(":{}", port_) : ""),
        curl_easy_strerror(result), http_code, server_message));
  }
  curl_slist_free_all(headerlist);
  curl_easy_cleanup(curl);

  /* At this point we have a seemingly valid return file from the server, see
   if it can be loaded as one of the supported image types.  We do not check for
   the validity (e.g., underlying type or number of channels), the Load*Image
   methods are responsible for this.  However, this method will rename the file
   from ".bin" to the correct image file extension for better housekeeping in
   the temp_directory_.

   If the server did not return one of the kinds of files that are supported,
   error out now.

   NOTE: do not rely on or trust the server to (correctly) report a valid mime
   type for the sent image.  VTK's image readers 'CanReadFile' methods check
   if the file *content* can actually be loaded (regardless of extension). */
  std::string image_types_tried = "";  // Build up for error message at end.

  vtkNew<vtkPNGReader> png_reader;
  if (png_reader->CanReadFile(bin_out_path.c_str())) {
    return RenameFileExtension(bin_out_path, ".png");
  }
  image_types_tried += "PNG";

  vtkNew<vtkTIFFReader> tiff_reader;
  if (tiff_reader->CanReadFile(bin_out_path.c_str())) {
    return RenameFileExtension(bin_out_path, ".tiff");
  }
  image_types_tried += ", TIFF";

  throw std::runtime_error(fmt::format(
      "RenderClient: while trying to render the scene_id={} with provided "
      "scene_path='{}', the file returned by the server saved in '{}' is not "
      "understood as an image type that is supported.  Image types attempted "
      "loading as: {}.",
      scene_sha256, scene_path, bin_out_path, image_types_tried));
}

std::string RenderClient::ComputeSha256(const std::string& path) const {
  try {
    std::ifstream f_in(path, std::ios::binary);
    std::vector<unsigned char> hash(picosha2::k_digest_size);
    picosha2::hash256(f_in, hash.begin(), hash.end());
    return picosha2::bytes_to_hex_string(hash.begin(), hash.end());
  } catch (const std::exception& e) {
    throw std::runtime_error("ComputeSha256: unable to compute hash: " +
                             std::string(e.what()));
  }
}

void RenderClient::ValidDepthRangeOrThrow(double min_depth,
                                          double max_depth) const {
  // Make sure min_depth/max_depth are provided and make sense for depth images.
  if (min_depth < 0.0 || max_depth < 0.0)
    throw std::logic_error(
        "min_depth and max_depth must be provided for "
        "depth images, and be positive.");
  if (max_depth <= min_depth)
    throw std::logic_error(
        "max_depth cannot be less than or equal to min_depth.");
}

std::string RenderClient::RenameFileExtension(const std::string& path,
                                              const std::string& ext) const {
  const drake::filesystem::path origin{path};
  drake::filesystem::path destination{path};
  destination.replace_extension(drake::filesystem::path{ext});
  drake::filesystem::rename(origin, destination);

  // TODO(svenevs): logging desired?  Replace with log() from drake?
  if (verbose_) {
    std::cout << "RenderClient: renamed " << origin << " to " << destination
              << '\n';
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
        "RenderClient: loaded PNG image from '{}' has {} channels, but either "
        "3 (RGB) or 4 (RGBA) are required for color images.",
        path, channels));
  }

  // Make sure we have a standard PNG image with uint8_t data per channel.
  if (image_data->GetScalarType() != VTK_UNSIGNED_CHAR) {
    throw std::runtime_error(fmt::format(
        "RenderClient: loaded PNG image from '{}' has a channel size in bytes "
        "of {}, but only RGB and RGBA uchar (channel size=1) images are "
        "supported.",
        path, image_data->GetScalarSize()));
  }

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
        const int in_idx = (x + (height - y) * width) * kNumChannels;
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
        const int in_idx = (x + (height - y) * width) * 3;
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
  if (image_data->GetScalarType() != VTK_TYPE_FLOAT32) {
    throw std::runtime_error(fmt::format(
        "RenderClient: loaded TIFF image from '{}' did not have floating point "
        "data, but float TIFF is required for depth images.",
        path));
  }

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

  if (image_data->GetScalarType() != VTK_TYPE_UINT16) {
    throw std::runtime_error(fmt::format(
        "RenderClient: loaded PNG image from '{}' did not have ushort data, "
        "but single channel ushort PNG is required for label images.",
        path));
  }

  /* NOTE: Officially label image is signed integers, the vtkImageExport::Export
   will reinterpret this as unsigned internally, since the loaded PNG image is
   required to be unsigned short data, negative values will not occur. */
  image_exporter->Export(label_image_out->at(0, 0));
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
