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
  /* MemoryFileAndBackreference tacks on a weak_ptr<Impl> to a database
  MemoryFile. When this object is destroyed it will erase the MemoryFile from
  the impl's map. (unless the Impl has already been destroyed). */
  struct MemoryFileAndBackreference {
    ~MemoryFileAndBackreference() {
      if (std::shared_ptr<FileStorage::Impl> impl = backreference.lock()) {
        std::lock_guard guard(impl->mutex);
        auto iter = impl->map.find(memory_file.sha256());
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

    MemoryFile memory_file;
    std::weak_ptr<Impl> backreference;
  };

  // The mutex_ guards all other fields.
  // TODO(jwnimmer-tri) We could use an off the shelf concurrent hash map
  // and avoid the need for a map-wide mutex.
  mutable std::mutex mutex;
  std::unordered_map<Sha256, std::weak_ptr<MemoryFile>> map;
};

FileStorage::FileStorage() : impl_(std::make_shared<Impl>()) {}

FileStorage::~FileStorage() = default;

std::shared_ptr<const MemoryFile> FileStorage::Insert(
    std::string&& content, std::string&& filename_hint) {
  // Remove any newlines from the filename_hint.
  for (auto& ch : filename_hint) {
    if (ch == '\n' || ch == '\r') {
      ch = ' ';
    }
  }

  // Take ownership of the provided strings (cleaning them out) and computing
  // the sha outside the critical section.
  // Note: meshcat file storage doesn't depend on knowing the extension of the
  // files being inserted, so we'll simply provide an empty extension.
  MemoryFile new_memory_file(std::move(content), "", std::move(filename_hint));

  // Hold a transactional lock for all operations on our map.
  // This is important to avoid TOCTOU races.
  std::lock_guard guard(impl_->mutex);

  // Attempt to re-use an existing entry.
  if (std::shared_ptr<const MemoryFile> old_memory_file =
          FindWhileLocked(new_memory_file.sha256())) {
    return old_memory_file;
  }

  // We'll be returning a shared_ptr<const MemoryFile> but what we allocate here
  // is actually a shared_ptr<MemoryFileAndBackReference>, so that when the use
  // count hits zero it will remove itself from our unordered_map. That means
  // calling the so-called "aliasing" constructor for shared_ptr<MemoryFile>.
  auto fat_memory_file = std::make_shared<Impl::MemoryFileAndBackreference>();
  fat_memory_file->backreference = impl_;
  fat_memory_file->memory_file = std::move(new_memory_file);

  std::shared_ptr<MemoryFile> file(fat_memory_file,
                                   &(fat_memory_file->memory_file));

  impl_->map[file->sha256()] = file;

  return file;
}

std::shared_ptr<const MemoryFile> FileStorage::Find(
    const Sha256& sha256) const {
  std::lock_guard guard(impl_->mutex);
  return FindWhileLocked(sha256);
}

std::shared_ptr<const MemoryFile> FileStorage::FindWhileLocked(
    const Sha256& sha256) const {
  auto iter = impl_->map.find(sha256);
  if (iter != impl_->map.end()) {
    const std::weak_ptr<MemoryFile>& weak_memory_file = iter->second;
    return weak_memory_file.lock();
  }
  return {};
}

std::vector<std::shared_ptr<const MemoryFile>> FileStorage::DumpEverything()
    const {
  std::vector<std::shared_ptr<const MemoryFile>> result;
  {
    // Use a critical section to copy out the map.
    std::lock_guard guard(impl_->mutex);
    result.reserve(impl_->map.size());
    for (const auto& [_, weak_memory_file] : impl_->map) {
      std::shared_ptr<const MemoryFile> memory_file = weak_memory_file.lock();
      if (memory_file != nullptr) {
        result.push_back(std::move(memory_file));
      }
    }
  }
  // For determinism, sort the result (but outside of the critical section).
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a->sha256() < b->sha256();
  });
  return result;
}

size_t FileStorage::size() const {
  std::lock_guard guard(impl_->mutex);
  return impl_->map.size();
}

std::string FileStorage::GetCasUrl(const MemoryFile& asset) {
  // This URL pattern must align with HandleHttpGet() in `meshcat.cc`, so if you
  // change it be sure to change both places. It's also important that it does
  // not contain a subdirectory (i.e., no "/" characters) because that confuses
  // any assets that refer to sub-assets. We use "v1" as in "version 1"; if we
  // ever change the URL semantics in the future (e.g., if we find a bug where
  // we were serving incomplete files) we will need to change the URL version
  // salt so that clients will re-fetch the files.
  return fmt::format("cas-v1/{}", asset.sha256().to_string());
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
