#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_optional.h"

namespace drake {


optional<std::string> loaded_library_path(const std::string &library_name);


}  // namespace drake
