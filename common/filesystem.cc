#include "drake/common/filesystem.h"

#include <sys/stat.h>
#include <unistd.h>

#include <stdexcept>

#include "fmt/format.h"

namespace drake {
namespace internal {

bool IsFile(const std::string& filesystem_path) {
  struct stat attributes{};
  int error = stat(filesystem_path.c_str(), &attributes);
  if (error) { return false; }
  return S_ISREG(attributes.st_mode);
}

bool IsDir(const std::string& filesystem_path) {
  struct stat attributes{};
  int error = stat(filesystem_path.c_str(), &attributes);
  if (error) { return false; }
  return S_ISDIR(attributes.st_mode);
}

std::string Readlink(const std::string& pathname) {
  std::string result;
  result.resize(4096);
  ssize_t length = ::readlink(pathname.c_str(), &result.front(), result.size());
  if (length < 0) {
    throw std::runtime_error(fmt::format(
        "Could not open {}", pathname));
  }
  if (length >= static_cast<ssize_t>(result.size())) {
    throw std::runtime_error(
        fmt::format("Could not readlink {} (too long)", pathname));
  }
  result.resize(length);
  return result;
}

}  // namespace internal
}  // namespace drake
