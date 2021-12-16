#include "drake/geometry/render/dev/render_client_gltf.h"

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <picosha2.h>
#include <vtkGLTFExporter.h>
#include <vtkImageExport.h>
#include <vtkPNGReader.h>

namespace drake {
namespace geometry {
namespace render {

using std::filesystem::path;
using json = nlohmann::json;

using systems::sensors::ImageRgba8U;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;

namespace {

// Write callback for libcurl, assumes `userp` points to an std::string.
// See: https://curl.se/libcurl/c/libcurl-tutorial.html
size_t write_string_data(void* buffer, size_t size, size_t nmemb, void* userp) {
  const size_t data_size = size * nmemb;
  std::string* s = static_cast<std::string*>(userp);
  s->append(static_cast<char*>(buffer), data_size);
  return data_size;
}

size_t write_file_data(void* buffer, size_t size, size_t nmemb, void* userp) {
  const size_t data_size = size * nmemb;
  std::ofstream* f = static_cast<std::ofstream*>(userp);
  f->write(static_cast<char*>(buffer), data_size);
  return data_size;
}

}  // namespace

RenderClientGLTF::RenderClientGLTF(const RenderClientGLTFParams& parameters)
    : RenderEngineVtk({parameters.default_label, parameters.default_diffuse,
                       parameters.default_clear_color}),
      RenderClient({parameters.default_label, parameters.url, parameters.port,
                    parameters.upload_endpoint, parameters.render_endpoint,
                    parameters.curl_verbose}) { }

RenderClientGLTF::RenderClientGLTF(const RenderClientGLTF& other)
    : RenderEngine(other), RenderEngineVtk(other), RenderClient(other) { }

std::unique_ptr<RenderEngine> RenderClientGLTF::DoClone() const {
  return std::unique_ptr<RenderClientGLTF>(
      new RenderClientGLTF(*this));
}

RenderClientGLTF::~RenderClientGLTF() {
}

void RenderClientGLTF::ExportColorImage(ImageRgba8U* buffer) const {
  // TODO(svenevs): is there a good place to track a global frame id?
  static size_t color_frame_id{0};
  ExportImage(ImageType::kColor, color_frame_id++, buffer);
}

void RenderClientGLTF::ExportDepthImage(ImageRgba8U* buffer) const {
  // TODO(svenevs): is there a good place to track a global frame id?
  static size_t depth_frame_id{0};
  ExportImage(ImageType::kDepth, depth_frame_id++, buffer);
}

void RenderClientGLTF::ExportLabelImage(ImageRgba8U* buffer) const {
  // TODO(svenevs): is there a good place to track a global frame id?
  static size_t label_frame_id{0};
  ExportImage(ImageType::kLabel, label_frame_id++, buffer);
}

std::string RenderClientGLTF::ExportPathFor(ImageType image_type,
                                            size_t frame_id) const {
  // Create e.g., {temp_directory_}/000000000000-color.gltf
  const path base{temp_directory_};
  const std::string frame{fmt::format("{:0>12}", frame_id)};
  std::string suffix;
  if (image_type == ImageType::kColor)
    suffix = "-color.gltf";
  else if (image_type == ImageType::kDepth)
    suffix = "-depth.gltf";
  else  // image_type == ImageType::kLabel
    suffix = "-label.gltf";
  return base / (frame + suffix);
}

void RenderClientGLTF::ExportImage(ImageType image_type, size_t frame_id,
                                   ImageRgba8U* buffer) const {
  // First, export the glTF file to disk.
  vtkNew<vtkGLTFExporter> gltf_exporter;
  gltf_exporter->InlineDataOn();
  gltf_exporter->SetRenderWindow(pipelines_[image_type]->window);
  const std::string scene_path = ExportPathFor(image_type, frame_id);
  gltf_exporter->SetFileName(scene_path.c_str());
  gltf_exporter->Write();

  const std::string scene_sha256 = ComputeSha256(scene_path);
  PostScene(image_type, scene_path, scene_sha256);
  std::string png_path = GetRender(image_type, scene_path, scene_sha256);

  // TODO(svenevs): share components of this code with
  //                geometry/render/gl_renderer/texture_library.cc?
  vtkNew<vtkPNGReader> png_reader;
  png_reader->SetFileName(png_path.c_str());

  // TODO(svenevs): what are our error path options?  Exceptions?  Skip?
  if (!png_reader->CanReadFile(png_path.c_str())) {
    std::cerr << "ERROR: cannot read png file: " << png_path << '\n';
    return;
  }  // this is not done in texture_library, but it checks similar things.

  png_reader->Update();
  vtkNew<vtkImageExport> image_exporter;
  image_exporter->SetInputConnection(png_reader->GetOutputPort());
  image_exporter->ImageLowerLeftOff();
  image_exporter->Update();
  vtkImageData* image = image_exporter->GetInput();

  // TODO(svenevs): error path options...
  if (image == nullptr) {
    // return std::nullopt;// texture_library return
    std::cerr << "ERROR: not able to load " << png_path << '\n';
    return;
  }

  const int* dim = image->GetDimensions();
  const int width = dim[0];
  const int height = dim[1];

  // Additional check not present in texture_library
  auto *wh = pipelines_[image_type]->window->GetSize();
  if (wh[0] != width || wh[1] != height) {
    std::cerr << "ERROR: expected (width=" << wh[0] << ", height=" << wh[1]
              << "), but got (width=" << width << ", height=" << height
              << ") for " << png_path << '\n';
    return;
  }

  // We should have either rgb or rgba data.
  const int num_channels = image->GetNumberOfScalarComponents();
  // TODO(svenevs): do we support just RGBA, or also RGB from server?
  // this check is different than texture_library
  if (num_channels != 4) {
    std::cerr << "ERROR: needed 4 image channels but got " << num_channels
              << " for image " << png_path << '\n';
    return;
  }
  // Each channel should be an unsigned byte.
  if (image->GetScalarType() != VTK_UNSIGNED_CHAR) {
    std::cerr << "ERROR: expected VTK_UNSIGNED_CHAR (code=" << VTK_UNSIGNED_CHAR
              << "), but got code=" << image->GetScalarType() << " for image "
              << png_path << '\n';
    return;
  }

  // Copy the loaded image to the desired render buffer.
  image_exporter->Export(buffer->at(0, 0));
}

std::string RenderClientGLTF::ComputeSha256(const std::string& path) const {
  std::ifstream gltf_in(path, std::ios::binary);
  std::vector<unsigned char> hash(picosha2::k_digest_size);
  picosha2::hash256(gltf_in, hash.begin(), hash.end());
  return picosha2::bytes_to_hex_string(hash.begin(), hash.end());
}

void RenderClientGLTF::PostScene(ImageType image_type,
                                 const std::string& scene_path,
                                 const std::string& scene_sha256) const {
  CURL *curl{nullptr};
  CURLcode result;
  curl = curl_easy_init();
  DRAKE_DEMAND(curl != nullptr);

  if (curl_verbose_)
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

  // Setup the POST url.
  const std::string upload_url = url_ + "/" + upload_endpoint_;
  curl_easy_setopt(curl, CURLOPT_URL, upload_url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  if (port_ > 0)
    curl_easy_setopt(curl, CURLOPT_PORT, port_);

  // Create and fill out a <form> to POST.
  curl_mime* form{nullptr};
  curl_mimepart* field{nullptr};
  struct curl_slist* headerlist{nullptr};
  form = curl_mime_init(curl);

  // Fill out <input type="file" name="data">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "data");
  curl_mime_filedata(field, scene_path.c_str());

  // TODO(svenevs): make a helper method for this.
  // Fill out <input type="text" name="image_type">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "image_type");
  std::string image_type_str;
  if (image_type == ImageType::kColor)
    image_type_str = "color";
  else if (image_type == ImageType::kDepth)
    image_type_str = "depth";
  else  // image_type == ImageType::kLabel
    image_type_str = "label";
  curl_mime_data(field, image_type_str.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="submit" value="Upload" name="submit">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "submit");
  curl_mime_data(field, "Upload", CURL_ZERO_TERMINATED);

  curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);

  // Disable 100-Coninue.  See:
  // http://www.iandennismiller.com/posts/curl-http1-1-100-continue-and-multipartform-data-post.html
  headerlist = curl_slist_append(headerlist, "Expect:");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);

  // Receive json response from the server.
  std::string post_response_text;
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &post_response_text);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &write_string_data);

  // Perform the POST.
  // TODO(svenevs): if we set FAILONERROR then we lose the json response.
  // Mark failed for 4xx and 5xx
  // curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
  result = curl_easy_perform(curl);

  // TODO(svenevs): in favor of CURLOPT_FAILONERROR, is this OK?
  int64_t http_code{0};
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  const bool http_code_ok = http_code >= 200 && http_code < 400;

  // TODO(svenevs): what do we do if curl fails?  Retry?  How many times?
  if (result != CURLE_OK || !http_code_ok) {
    std::cerr << "ERROR doing POST: /" << upload_endpoint_ << ":\n"
              << "  cURL Message:   " << curl_easy_strerror(result) << '\n'
              << "  HTTP Code:      " << http_code << '\n'
              << "  Server Message: " << post_response_text << '\n';
  }
  curl_slist_free_all(headerlist);
  curl_easy_cleanup(curl);

  // Validate the server response.
  // TODO(svenevs): exception handling here!  What if the sha256 is wrong?
  const json post_response = json::parse(post_response_text);
  const std::string server_sha256 = post_response["sha256"];

  // TODO(svenevs): change this into a log() call?  Or delete.
  std::cout << "==> POST /" << upload_endpoint_ << " complete:\n"
            << "    File: " << scene_path << '\n'
            << "    Expected sha256: " << scene_sha256 << '\n'
            << "    Server sha256:   " << server_sha256 << '\n';
}

std::string RenderClientGLTF::GetRender(ImageType image_type,
                                        const std::string& scene_path,
                                        const std::string& scene_sha256) const {
  CURL *curl{nullptr};
  CURLcode result;
  curl = curl_easy_init();
  DRAKE_DEMAND(curl != nullptr);

  if (curl_verbose_)
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

  // Setup the POST url.
  const std::string post_url = url_ + "/" + render_endpoint_;
  curl_easy_setopt(curl, CURLOPT_URL, post_url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  if (port_ > 0)
    curl_easy_setopt(curl, CURLOPT_PORT, port_);

  // Create and fill out a <form> to POST.
  curl_mime* form{nullptr};
  curl_mimepart* field{nullptr};
  struct curl_slist* headerlist{nullptr};
  form = curl_mime_init(curl);

  // Fill out <input type="text" name="id">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "id");
  curl_mime_data(field, scene_sha256.c_str(), CURL_ZERO_TERMINATED);

  // TODO(svenevs): make a helper method for this.
  // Fill out <input type="text" name="image_type">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "image_type");
  std::string image_type_str;
  if (image_type == ImageType::kColor)
    image_type_str = "color";
  else if (image_type == ImageType::kDepth)
    image_type_str = "depth";
  else  // image_type == ImageType::kLabel
    image_type_str = "label";
  curl_mime_data(field, image_type_str.c_str(), CURL_ZERO_TERMINATED);

  /* TODO(svenevs): is it valid to rely on vtk window width/height?  It gets
   updated in RenderEngineVtk::UpdateWindow which is called before all of this.
   Otherwise the refactor should be revisited to get access to RenderCameraCore
   again.
   */
  // Get the width / height of the window we are rendering.
  auto *wh = pipelines_[image_type]->window->GetSize();
  std::string width = fmt::format("{}", wh[0]);
  std::string height = fmt::format("{}", wh[1]);

  // Fill out <input type="number" name="width">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "width");
  curl_mime_data(field, width.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="number" name="height">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "height");
  curl_mime_data(field, height.c_str(), CURL_ZERO_TERMINATED);

  // Fill out <input type="submit" name="submit" value="Render">
  field = curl_mime_addpart(form);
  curl_mime_name(field, "submit");
  curl_mime_data(field, "Render", CURL_ZERO_TERMINATED);

  curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);

  // Disable 100-Coninue.  See:
  // http://www.iandennismiller.com/posts/curl-http1-1-100-continue-and-multipartform-data-post.html
  headerlist = curl_slist_append(headerlist, "Expect:");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);

  // Have curl write directly to a file
  // TODO(svenevs): error checking...
  const std::string png_out = scene_path + ".png";
  // image = fopen(png_out.c_str(), "wb");
  std::ofstream image(png_out, std::ios::binary);
  // TODO(svenevs): can we get json response?  I *think* the file ends up with
  // json when there is an error?
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &write_file_data);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &image);

  // Perform the POST.
  // TODO(svenevs): if we set FAILONERROR then we lose the json response.
  // Mark failed for 4xx and 5xx
  // curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
  result = curl_easy_perform(curl);

  // TODO(svenevs): in favor of CURLOPT_FAILONERROR, is this OK?
  int64_t http_code{0};
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  const bool http_code_ok = http_code >= 200 && http_code < 400;

  // TODO(svenevs): what do we do if curl fails?  Retry?  How many times?
  if (result != CURLE_OK || !http_code_ok) {
    // TODO(svenevs): it's not a guarantee that png_out got json in error...
    std::ifstream png_in(png_out);
    std::stringstream buff;
    if (png_in.is_open()) {
      buff << png_in.rdbuf();
    }
    // For whatever reason cout/cerr was not printing?
    throw std::runtime_error(fmt::format(R"(
        ERROR doing POST: /{}
          cURL Message:   {}
          HTTP Code:      {}
          Server Message: {}
        )",
        render_endpoint_, curl_easy_strerror(result), http_code, buff.str()));
  }
  curl_slist_free_all(headerlist);
  curl_easy_cleanup(curl);

  // TODO(svenevs): change this into a log() call?  Or delete.
  std::cout << "==> POST /" << render_endpoint_ << " complete:\n"
            << "    File: " << png_out << '\n';

  return png_out;
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
