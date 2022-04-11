#include "drake/geometry/render/dev/render_gltf_client/internal_http_service.h"

#include <fstream>
#include <vector>

#include <fmt/format.h>

#include "drake/common/filesystem.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace fs = drake::filesystem;

HttpService::HttpService() {}

HttpService::~HttpService() {}

void HttpService::ThrowIfUrlInvalid(const std::string& url) const {
  // Validate what can be validated about the provided url.
  if (url.empty()) {
    throw std::logic_error("HttpService: url parameter may not be empty.");
  }
  if (url.back() == '/') {
    throw std::logic_error("HttpService: url may not end with '/'.");
  }
}

void HttpService::ThrowIfEndpointInvalid(const std::string& endpoint) const {
  if (endpoint.size() >= 1) {
    if (endpoint.front() == '/' || endpoint.back() == '/') {
      throw std::runtime_error(fmt::format(
          "Provided endpoint='{}' is not valid, it may not start or end with "
          "a '/'.",
          endpoint));
    }
  }
}

void HttpService::ThrowIfFilesMissing(
    const std::map<std::string,
                   std::pair<std::string, std::optional<std::string>>>&
        file_fields) const {
  std::vector<std::string> missing_files;
  for (const auto& [field_name, field_data_pair] : file_fields) {
    const auto& file_path = field_data_pair.first;
    if (!fs::is_regular_file(file_path)) {
      missing_files.emplace_back(fmt::format("{}='{}'", field_name, file_path));
    }
  }
  if (missing_files.size() > 0) {
    std::string exc_message = "Provided file fields had missing file(s): ";
    for (size_t i = 0; i < missing_files.size(); ++i) {
      exc_message += missing_files[i];
      if (i < missing_files.size() - 1) exc_message += ", ";
    }
    exc_message += ".";
    throw std::runtime_error(exc_message);
  }
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
