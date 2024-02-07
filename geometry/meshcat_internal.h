#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/meshcat_file_storage_internal.h"

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

/* Rewrites a glTF file in memory so that all of its URIs refer to assets in
FileStorage, instead of their original location.

When a glTF file has bundled assets (i.e., data URIs) this decodes and unbundles
them. Meshcat is MUCH slower at loading bundled assets compared to unbundled.

When a glTF file has relative path URIs (i.e., unbundled files on disk), this
loads the files into FileStorage so that we can serve them later, even if the
original file has disappeared in the meantime.

@param[in] gltf_filename The glTF filename, used to calculate relative paths.

@param[in,out] gltf_contents The contents of `gltf_filename`. It will be edited
  in place to replace the URIs. (We assume that the caller has already read the
  file into a string, so here we can just operate on that string as an [in,out]
  parameter instead of re-reading the file and using an output-only parameter.)

@param[in,out] storage The database where assets should be stored.

@returns The handles for all assets cited by `gltf_contents`. */
[[nodiscard]] std::vector<std::shared_ptr<const FileStorage::Handle>>
UnbundleGltfAssets(const std::filesystem::path& gltf_filename,
                   std::string* gltf_contents, FileStorage* storage);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
