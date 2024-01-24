#include "drake/geometry/meshcat_internal.h"

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
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

struct FileStorage::Impl {
  struct HandleAndBackreference {
    ~HandleAndBackreference() {
      if (std::shared_ptr<FileStorage::Impl> impl = backreference.lock()) {
        std::lock_guard guard(impl->mutex);
        impl->map.erase(handle.sha256);
      }
    }

    FileStorage::Handle handle;
    std::weak_ptr<Impl> backreference;
  };

  // The mutex_ guards all other fields.
  // TODO(jwnimmer-tri) We could use an off the shelf concurrent hash map
  // and avoid the need for a map-wide mutex.
  mutable std::mutex mutex;
  std::unordered_map<Sha256, std::weak_ptr<Handle>> map;
};

FileStorage::FileStorage() : impl_(std::make_shared<Impl>()) {}

FileStorage::~FileStorage() = default;

std::shared_ptr<const FileStorage::Handle> FileStorage::Insert(
    std::string&& content, std::string&& filename_hint) {
  // Do the checksum outside of the critical section.
  const Sha256 sha256 = Sha256::Checksum(content);

  // Remove any newlines from the filename_hint.
  for (auto& ch : filename_hint) {
    if (ch == '\n' || ch == '\r') {
      ch = ' ';
    }
  }

  // Hold a transactional lock for all operations on our map.
  // This is important to avoid TOCTOU races.
  std::lock_guard guard(impl_->mutex);

  // Attept to re-use an existing entry.
  if (std::shared_ptr<const Handle> old_handle = FindWhileLocked(sha256)) {
    // No need to insert a new copy -- but for consistency we should
    // still dispose of the user-provided strings.
    content.clear();
    filename_hint.clear();
    return old_handle;
  }

  // We'll be returning a shared_ptr<const Handle> but what we allocate here is
  // actually a shared_ptr<HandleAndBackreference>, so that when the use count
  // hits zero it will remove itself from our unordered_map. That means calling
  // the so-called "aliasing" constructor for `handle`.
  auto fat_handle = std::make_shared<Impl::HandleAndBackreference>();
  fat_handle->backreference = impl_;
  std::shared_ptr<Handle> handle(fat_handle, &(fat_handle->handle));

  handle->content = std::move(content);
  handle->sha256 = sha256;
  handle->filename_hint = std::move(filename_hint);
  impl_->map[sha256] = handle;

  return handle;
}

std::shared_ptr<const FileStorage::Handle> FileStorage::Find(
    const Sha256& sha256) const {
  std::lock_guard guard(impl_->mutex);
  return FindWhileLocked(sha256);
}

std::shared_ptr<const FileStorage::Handle> FileStorage::FindWhileLocked(
    const Sha256& sha256) const {
  auto iter = impl_->map.find(sha256);
  if (iter != impl_->map.end()) {
    const std::weak_ptr<Handle>& weak_handle = iter->second;
    return weak_handle.lock();
  }
  return {};
}

std::vector<std::shared_ptr<const FileStorage::Handle>>
FileStorage::DumpEverything() const {
  std::vector<std::shared_ptr<const Handle>> result;
  {
    // Use a critical section to copy out the map.
    std::lock_guard guard(impl_->mutex);
    result.reserve(impl_->map.size());
    for (const auto& [sha256, weak_handle] : impl_->map) {
      std::shared_ptr<const Handle> handle = weak_handle.lock();
      if (handle != nullptr) {
        result.push_back(std::move(handle));
      }
    }
  }
  // For determinism, sort the result (but outside of the critical section).
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a->sha256 < b->sha256;
  });
  return result;
}

size_t FileStorage::size() const {
  std::lock_guard guard(impl_->mutex);
  return impl_->map.size();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
