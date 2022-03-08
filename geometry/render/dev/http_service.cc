#include "drake/geometry/render/dev/http_service.h"

#include <fstream>
#include <vector>

#include <fmt/format.h>

#include "drake/common/filesystem.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

namespace fs = drake::filesystem;

namespace {

/* Throw an std::runtime_error if the directory does not exist, or cannot be
 written to / read from. */
void ThrowIfInvalidTempDirectory(const std::string& temp_directory) {
  // First, check if it is a directory.
  const auto temp_dir_path = fs::path(temp_directory);
  if (!fs::is_directory(temp_dir_path)) {
    throw std::runtime_error(fmt::format(
        "HttpService: provided temp directory '{}' is not a directory.",
        temp_directory));
  }

  /* Next, check that we can write to / read from the directory.  Validating the
   permissions of a folder for write and read access is sufficiently complex
   enough that the most straightforward solution is to simply write a temporary
   file and read it back.  Any exceptions raised imply we do not have access. */
  try {
    const std::string test_txt = temp_dir_path / "http_service_write_test.txt";
    std::ofstream out(test_txt, std::ios::binary);
    out << "test!\n";
    out.close();
    std::ifstream in(test_txt, std::ios::binary);
    std::string text;
    while (std::getline(in, text)) {
      break;
    }
    in.close();
    fs::remove(test_txt);
  } catch (const std::exception&) {
    throw std::runtime_error(fmt::format(
        "HttpService: cannot write to / read from '{}'.", temp_directory));
  }
}

/* Throw an std::logic_error if the provided url is empty or has trailing
 slashes. */
void ThrowIfInvalidUrl(const std::string& url) {
  // Validate what can be validated about the provided url.
  if (url.empty()) {
    throw std::logic_error("HttpService: url parameter may not be empty.");
  }
  if (url.back() == '/') {
    throw std::logic_error("HttpService: url may not end with '/'.");
  }
}

}  // namespace

HttpService::HttpService(const std::string& temp_directory,
                         const std::string& url, int32_t port, bool verbose)
    : temp_directory_{temp_directory},
      url_{url},
      port_{port},
      verbose_{verbose} {
  ThrowIfInvalidTempDirectory(temp_directory_);
  ThrowIfInvalidUrl(url_);
}

HttpService::HttpService(const HttpService& other)
    : temp_directory_{other.temp_directory_},
      url_{other.url_},
      port_{other.port_},
      verbose_{other.verbose_} {}

std::unique_ptr<HttpService> HttpService::Clone() const {
  std::unique_ptr<HttpService> clone(DoClone());
  // Make sure that derived classes have actually overridden DoClone().
  // Particularly important for derivations of derivations.
  // Note: clang considers typeid(*clone) to be an expression with side effects.
  // So, we capture a reference to the polymorphic type and provide that to
  // typeid to make both clang and gcc happy.
  const HttpService& clone_ref = *clone;
  if (typeid(*this) != typeid(clone_ref)) {
    throw std::logic_error(fmt::format(
        "Error in cloning HttpService class of type {}; the clone returns "
        "type {}. {}::DoClone() was probably not implemented",
        NiceTypeName::Get(*this), NiceTypeName::Get(clone_ref),
        NiceTypeName::Get(*this)));
  }
  return clone;
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
}  // namespace render
}  // namespace geometry
}  // namespace drake
