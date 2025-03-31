#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/memory_file.h"
#include "drake/common/sha256.h"

namespace drake {
namespace geometry {
namespace internal {

/* FileStorage maintains an in-memory database with the contents of files, keyed
by their checksum.

For large assets (e.g., images) sometimes we use the msgpack message to transmit
the URL of the asset, instead of the asset as inline data. In that case, we need
to store the asset so that we can serve it over HTTP GET when a browser client
needs it. Our storage must checkpoint the asset in memory -- we can't just keep
around the fs::path and re-read it again later because the file might have
disappeared or changed in the meantime.

This storage is built around shared_ptr and weak_ptr so that users of this class
are responsible for managing the lifetime of the assets. The FileStorage class
only holds weak pointers. The Insert() function returns a shared_ptr to the
caller, who is responsible for keeping that shared_ptr around exactly as long as
the asset should remain in the database. Inserting the same asset again will
bump the use_count (by returning a copy of the same shared_ptr).

Because this class is accessed by both the Meshcat main thread (to insert
assets) and the websocket worker thread (to serve assets), this class is
thread-safe: all functions other than the constructor and destructor take a
mutex lock. */
class FileStorage final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FileStorage);

  /* Constructs an empty database. */
  FileStorage();

  /* Clears the database; it does not destroy any MemoryFile objects still held
  by users outside of this class. You can think of it like only the database's
  index is destroyed. */
  ~FileStorage();

  /* Adds the given `content` to the database. Both arguments are consumed by
  this call and will be empty afterwards. (Use the return value instead.) If
  possible, this function re-uses an existing entry, i.e., this function will
  "intern" the content.

  The `filename_hint` is purely a debugging aid (e.g., for logging), and is
  allowed to be empty; any newlines in the hint will be replaced. When the
  content already existed in the cache, the new hint will be dropped -- the hint
  used during the first first Insert() always wins. */
  [[nodiscard]] std::shared_ptr<const MemoryFile> Insert(
      std::string&& content, std::string&& filename_hint);

  /* Returns the database content with the given checksum, or when not found
  returns nullptr. */
  [[nodiscard]] std::shared_ptr<const MemoryFile> Find(
      const Sha256& sha256) const;

  /* Returns the entire database. All of the returned handles are non-null. */
  [[nodiscard]] std::vector<std::shared_ptr<const MemoryFile>> DumpEverything()
      const;

  /* Returns the number of files (i.e., MemoryFile instances) being stored. */
  [[nodiscard]] size_t size() const;

  /* Meshcat uses content-addressable storage ("CAS") to serve asset files. See
  https://en.wikipedia.org/wiki/Content-addressable_storage. This function
  returns the CAS URL for the given storage asset. */
  static std::string GetCasUrl(const MemoryFile& asset);

 private:
  /* The implementation of Find(). Assumes that `mutex_` is already held. */
  [[nodiscard]] std::shared_ptr<const MemoryFile> FindWhileLocked(
      const Sha256& sha256) const;

  // We need to use shared_ptr<Impl> so that the handles give out can can safely
  // keep pointers back to the Impl to remove themselves when they expire.
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
