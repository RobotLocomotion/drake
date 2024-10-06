#include "drake/common/file_source.h"

#include <fmt/format.h>

#include "drake/common/fmt_ostream.h"
#include "drake/common/overloaded.h"

namespace drake {
std::string to_string(const FileSource& source) {
  // TODO(jwnimmer-tri) Once we have a new enough fmt with `std.h`, a
  // simple `return fmt::to_string(source)` here will suffice.
  return std::visit(overloaded{[](const std::filesystem::path& path) {
                                 // We'll delegate to the built-in operator<<,
                                 // which properly quotes the path.
                                 return fmt::to_string(fmt_streamed(path));
                               },
                               [](const MemoryFile& file) {
                                 return fmt::to_string(file);
                               }},
                    source);
}

}  // namespace drake
