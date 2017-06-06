// This file is the Bazel build's implementation of drake_path.h.
#include "drake/common/drake_path.h"

namespace drake {

std::string GetDrakePath() {
  // TODO(jwnimmer-tri) GetDrakePath has a long and storied history (#1471,
  // #2174).  It serves multiple purposes (unit tests loading their models;
  // installed demos loading their models; etc.) but doesn't really do any of
  // them well.  This implementation is intended as a temporary shim, in order
  // to support Bazel and CMake build systems concurrently.
  //
  // TODO(jwnimmer-tri) This implementation only works when running inside the
  // sandbox, or when CWD is drake-distro.
  return std::string("drake");
}

}  // namespace drake
