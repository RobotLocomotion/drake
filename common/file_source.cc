#include "drake/common/file_source.h"

#include <fmt/format.h>

#include "drake/common/overloaded.h"

namespace drake {
std::string to_string(const FileSource& source) {  
  // Note: This would not be appropriate to use in the python __repr__
  // implementation because MemoryFile doesn't properly escape the appropriate
  // characters in the strings.
  return std::visit(overloaded{[](const std::filesystem::path& path) {
                                 return path.string();
                               },
                               [](const MemoryFile& file) {
                                 return file.to_string();
                               }},
                    source);
}

}  // namespace drake
