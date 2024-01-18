#include "drake/geometry/meshcat_internal.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <utility>

#include <fmt/format.h>

#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"

namespace fs = std::filesystem;

namespace drake {
namespace geometry {
namespace internal {

namespace {

std::string LoadResource(const std::string& resource_name) {
  const std::string resource = FindResourceOrThrow(resource_name);
  std::optional<std::string> content = ReadFile(resource);
  if (!content) {
    throw std::runtime_error(
        fmt::format("Error opening resource: {}", resource_name));
  }
  return std::move(*content);
}

}  //  namespace

std::optional<std::string_view> GetMeshcatStaticResource(
    std::string_view url_path) {
  static const drake::never_destroyed<std::string> meshcat_js(
      LoadResource("drake/geometry/meshcat.js"));
  static const drake::never_destroyed<std::string> stats_js(
      LoadResource("drake/geometry/stats.min.js"));
  static const drake::never_destroyed<std::string> meshcat_ico(
      LoadResource("drake/geometry/meshcat.ico"));
  static const drake::never_destroyed<std::string> meshcat_html(
      LoadResource("drake/geometry/meshcat.html"));
  if ((url_path == "/") || (url_path == "/index.html") ||
      (url_path == "/meshcat.html")) {
    return meshcat_html.access();
  }
  if (url_path == "/meshcat.js") {
    return meshcat_js.access();
  }
  if (url_path == "/stats.min.js") {
    return stats_js.access();
  }
  if (url_path == "/favicon.ico") {
    return meshcat_ico.access();
  }
  return {};
}

FileStorage::FileStorage() = default;

FileStorage::~FileStorage() = default;

FileStorage::Handle FileStorage::Insert(std::string&& content) {
  // Do the checksum outside of the critical section.
  const Sha256 sha256 = Sha256::Checksum(content);

  // Try to re-use an existing entry if possible.
  std::lock_guard guard(mutex_);
  Handle result = FindWhileLocked(sha256);
  if (result.content != nullptr) {
    // Found the content; no need to insert a new copy -- but for consistency we
    // should still dispose of the user-provided `content`.
    content.clear();
  } else {
    // Not found; we need to insert the content.
    auto content_ptr = std::make_shared<std::string>(std::move(content));
    map_.emplace(sha256, Item{.content = content_ptr});
    result.content = std::move(content_ptr);
    result.sha256 = sha256;
  }
  return result;
}

FileStorage::Handle FileStorage::Find(const Sha256& sha256) const {
  std::lock_guard guard(mutex_);
  return FindWhileLocked(sha256);
}

FileStorage::Handle FileStorage::FindWhileLocked(const Sha256& sha256) const {
  auto iter = map_.find(sha256);
  if (iter != map_.end()) {
    const Item& item = iter->second;
    std::shared_ptr<const std::string> strong = item.content.lock();
    if (strong != nullptr) {
      return {.sha256 = sha256, .content = std::move(strong)};
    }
  }
  return {};
}

std::vector<FileStorage::Handle> FileStorage::DumpEverything() const {
  std::vector<Handle> result;
  {
    // Use a critical section to copy out the map.
    std::lock_guard guard(mutex_);
    for (const auto& [sha256, item] : map_) {
      std::shared_ptr<const std::string> strong = item.content.lock();
      if (strong != nullptr) {
        result.emplace_back(
            Handle{.sha256 = sha256, .content = std::move(strong)});
      }
    }
  }
  // For determinism, sort the result (but outside of the critical section).
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a.sha256 < b.sha256;
  });
  return result;
}

int FileStorage::ShrinkToFit() {
  int result = 0;
  std::lock_guard guard(mutex_);
  for (auto iter = map_.begin(); iter != map_.end();) {
    Item& item = iter->second;
    if (item.content.expired()) {
      iter = map_.erase(iter);
      ++result;
    } else {
      ++iter;
    }
  }
  return result;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
