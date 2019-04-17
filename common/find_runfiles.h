#pragma once

#include <string>

namespace drake {
namespace internal {

bool HasRunfiles();

struct RlocationOrError {
  std::string rlocation_abspath;
  std::string error;
};

RlocationOrError FindRunfile(const std::string& resource_path);

}  // namespace internal
}  // namespace drake
