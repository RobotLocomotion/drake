#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/sha256.h"

namespace drake {
namespace geometry {
namespace internal {

/* Returns the static content for the given URL, or nullopt when the URL is
invalid. The valid static resource URLs are:
- `/`
- `/favicon.ico`
- `/index.html`
- `/meshcat.html`
- `/meshcat.js`
- `/stats.min.js` */
std::optional<std::string_view> GetMeshcatStaticResource(
    std::string_view url_path);

/* UuidGenerator generates random UUIDs:
https://en.wikipedia.org/wiki/Universally_unique_identifier#Version_4_(random)

This object is stateful so that each UUID will be distinct; the intended use is
to create one long-lived instance that services all requests for the lifetime of
the process.

Note that the UUIDs are *deterministically* random -- the i'th random UUID will
be the same from one run to the next. There is no re-seeding. */
class UuidGenerator final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UuidGenerator);

  UuidGenerator();
  ~UuidGenerator();

  /* Returns a newly-generated random UUID. */
  std::string GenerateRandom();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

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

  /* The record type contained in this database, as returned by Insert() or
  Find(). It is the responsibility of users of FileStorage to hold the `content`
  pointer (or the entire Handle) as long as the database entry should remain
  findable in the FileStorage. */
  struct Handle {
    /* The checksum of `content`. */
    Sha256 sha256;

    /* Some notional filename for the `content`, for use only in debugging.
    This is allowed to be empty. Must not contain any newlines. */
    std::string filename_hint;

    /* The (read-only) contents of the file. */
    std::shared_ptr<const std::string> content;
  };

  /* Constructs an empty database. */
  FileStorage();

  /* Clears the database; it does not destroy any reference-counted "content"
  strings or `Handle` objects still held by users outside of this class. You can
  think of it like only the database's index is destroyed. */
  ~FileStorage();

  /* Adds the given `content` to the database. Both arguments are consumed by
  this call and will be empty afterwards. (Use the return value instead.) If
  possible, this function re-uses an existing entry, i.e., this function will
  "intern" the content.

  The `filename_hint` is purely a debugging aid (e.g., for logging), and is
  allowed to be empty; any newlines in the hint will be replaced. When the
  content already existed in the cache, the new hint will be dropped; the hint
  used during the first first Insert() always wins. */
  [[nodiscard]] Handle Insert(std::string&& content,
                              std::string&& filename_hint);

  /* Returns the database content with the given checksum, or when not found
  returns a default-constructed Handle (with content == nullptr). */
  [[nodiscard]] Handle Find(const Sha256& sha256) const;

  /* Returns the entire database (not including any expired items).
  In other words, all of the handles are non-null. */
  [[nodiscard]] std::vector<Handle> DumpEverything() const;

  /* Reclaims unused memory.
  Returns the number of reclamations (for unit testing). */
  int ShrinkToFit();

 private:
  struct Item {
    /* If `content` has expired, returns a default-constructed Handle.
    Otherwise, copies this (plus the checksum) to a Handle and returns it. */
    Handle ToHandle(const Sha256& sha256) const;

    std::string filename_hint;
    std::weak_ptr<const std::string> content;

    // TODO(jwnimmer-tri) In the future, we might want to track additional
    // metadata in the Item record, e.g., mime type, etc.
  };

  /* The implementation of Find(). Assumes that `mutex_` is already held. */
  [[nodiscard]] Handle FindWhileLocked(const Sha256& sha256) const;

  // The mutex_ guards all other member fields.
  mutable std::mutex mutex_;
  std::unordered_map<Sha256, Item> map_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
