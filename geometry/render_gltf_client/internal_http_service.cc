#include "drake/geometry/render_gltf_client/internal_http_service.h"

#include <filesystem>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {
namespace {

void ThrowIfFilesMissing(const FileFieldsMap& file_fields) {
  std::vector<std::string> missing_files;
  for (const auto& [field_name, field_data_pair] : file_fields) {
    const auto& file_path = field_data_pair.first;
    if (!std::filesystem::is_regular_file(file_path)) {
      missing_files.emplace_back(fmt::format("{}='{}'", field_name, file_path));
    }
  }

  if (missing_files.size() > 0) {
    throw std::runtime_error(fmt::format(
        "RenderClient: provided file fields had missing file(s): {}.",
        fmt::join(missing_files, ", ")));
  }
}

}  // namespace

HttpService::~HttpService() = default;

HttpResponse HttpService::PostForm(const std::string& temp_directory,
                                   const std::string& url,
                                   const DataFieldsMap& data_fields,
                                   const FileFieldsMap& file_fields,
                                   bool verbose) {
  ThrowIfFilesMissing(file_fields);
  return DoPostForm(temp_directory, url, data_fields, file_fields, verbose);
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
