// This file is the Bazel build's implementation of drake_path.h.
#include "drake/common/drake_path.h"

#include <spruce.hh>

#include "drake/common/drake_throw.h"
#include "drake/common/find_resource.h"

namespace drake {

std::string GetDrakePath() {
  // Find something that represents where Drake lives.
  const auto& find_result = FindResource("drake/common/drake_path.sentinel");
  const std::string resource_path = find_result.get_absolute_path_or_throw();

  // Chop off two path elements to get back to drake-distro/drake.
  spruce::path working_path = resource_path;  // Set to "/foo/drake/common".
  working_path = working_path.root();         // Set to "/foo/drake".
  working_path = working_path.root();         // Set to "/foo".
  DRAKE_THROW_UNLESS(working_path.isDir());
  return working_path.getStr();
}

}  // namespace drake
