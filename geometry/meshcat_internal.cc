#include "drake/geometry/meshcat_internal.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include <fmt/format.h>
#include <uuid.h>

#include "drake/common/drake_export.h"
#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"

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

// We need an Impl for this class for two reasons:
// - The mt19937 object is ginormous and should not be inline.
// - The uuids object must be wrapped within DRAKE_NO_EXPORT.
struct DRAKE_NO_EXPORT UuidGenerator::Impl {
  std::mt19937 prng_;
  uuids::uuid_random_generator uuid_{prng_};
};

UuidGenerator::UuidGenerator() : impl_(std::make_unique<Impl>()) {}

UuidGenerator::~UuidGenerator() = default;

std::string UuidGenerator::GenerateRandom() {
  return uuids::to_string(impl_->uuid_());
}

FileStorage::FileStorage() = default;

FileStorage::~FileStorage() = default;

FileStorage::Handle FileStorage::Insert(std::string&& content,
                                        std::string&& filename_hint) {
  // Do the checksum outside of the critical section.
  const Sha256 sha256 = Sha256::Checksum(content);

  // Remove any newlines from the filename_hint.
  for (auto& ch : filename_hint) {
    if (ch == '\n' || ch == '\r') {
      ch = ' ';
    }
  }

  // Try to re-use an existing entry if possible.
  std::lock_guard guard(mutex_);
  Handle result = FindWhileLocked(sha256);
  if (result.content != nullptr) {
    // Found the content; no need to insert a new copy -- but for consistency we
    // should still dispose of the user-provided strings.
    content.clear();
    filename_hint.clear();
  } else {
    // Not found; we need to insert the content.
    auto content_ptr = std::make_shared<std::string>(std::move(content));
    map_[sha256] = Item{.filename_hint = filename_hint, .content = content_ptr};
    result.sha256 = sha256;
    result.filename_hint = std::move(filename_hint);
    result.content = std::move(content_ptr);
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
    return item.ToHandle(sha256);
  }
  return {};
}

FileStorage::Handle FileStorage::Item::ToHandle(const Sha256& sha256) const {
  std::shared_ptr<const std::string> strong = content.lock();
  return (strong != nullptr)
             ? FileStorage::Handle{.sha256 = sha256,
                                   .filename_hint = filename_hint,
                                   .content = std::move(strong)}
             : FileStorage::Handle{};
}

std::vector<FileStorage::Handle> FileStorage::DumpEverything() const {
  std::vector<Handle> result;
  {
    // Use a critical section to copy out the map.
    std::lock_guard guard(mutex_);
    result.reserve(map_.size());
    for (const auto& [sha256, item] : map_) {
      Handle handle = item.ToHandle(sha256);
      if (handle.content != nullptr) {
        result.push_back(std::move(handle));
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
