#include "drake/geometry/meshcat_file_storage_internal.h"

#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <utility>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace internal {

struct FileStorage::Impl {
  /* HandleAndBackreference tacks on a weak_ptr<Impl> to a database Handle.
  When this object is destroyed it will erase the Handle from the impl's map.
  (unless the Impl has already been destroyed). */
  struct HandleAndBackreference {
    ~HandleAndBackreference() {
      if (std::shared_ptr<FileStorage::Impl> impl = backreference.lock()) {
        std::lock_guard guard(impl->mutex);
        auto iter = impl->map.find(handle.sha256);
        if (iter != impl->map.end()) {
          // Only erase the map entry when it's use_count is actually zero. In
          // case another thread has re-inserted our same content while we were
          // waiting for the lock, the map might actually have a non-expired
          // pointer by the time we get here.
          if (iter->second.expired()) {
            impl->map.erase(iter);
          }
        }
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

  // Attempt to re-use an existing entry.
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
    for (const auto& [_, weak_handle] : impl_->map) {
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

std::string FileStorage::GetCasUrl(const FileStorage::Handle& asset) {
  // This URL pattern must align with HandleHttpGet() in `meshcat.cc`, so if you
  // change it be sure to change both places. It's also important that it does
  // not contain a subdirectory (i.e., no "/" characters) because that confuses
  // any assets that refer to sub-assets. We use "v1" as in "version 1"; if we
  // ever change the URL semantics in the future (e.g., if we find a bug where
  // we were serving incomplete files) we will need to change the URL version
  // salt so that clients will re-fetch the files.
  return fmt::format("cas-v1-{}", asset.sha256.to_string());
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
