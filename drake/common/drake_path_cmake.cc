// This file is the CMake build's implementation of drake_path.h.
/* clang-format off to disable clang-format-includes */
#include "drake/common/drake_path.h"
/* clang-format on */

#include "drake/common/cmake_project_source_dir.h"

namespace drake {

std::string GetDrakePath() {
  return detail::GetCMakeProjectSourceDir();
}

}  // namespace drake
