#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {
namespace internal {

/* Factory that assembles a `*.zip` file in memory. */
class MeshcatZipFactory final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatZipFactory);

  MeshcatZipFactory();

  ~MeshcatZipFactory();

  /* Adds the given `contents` to this `*.zip` using the given file `name`. */
  void AddFile(std::string_view name, std::string contents);

  /* Builds and returns the `*.zip` of all files added so far.
  @throws std::exception if AddFile() hasn't been called at least once. */
  [[nodiscard]] std::string Build() const;

 private:
  struct Entry {
    std::string name;
    std::string contents;
  };
  std::vector<Entry> entries_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
