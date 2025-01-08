#include "drake/common/cpu_capabilities.h"

#include <cstdlib>
#include <optional>
#include <string_view>
#include <vector>

#include "hwy/detect_targets.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace internal {
namespace {

std::string_view GetDisableCpuFeaturesEnvVar() {
  const char* const drake_env = std::getenv("DRAKE_DISABLE_CPU_FEATURES");
  if (drake_env != nullptr) {
    return std::string_view{drake_env};
  }
  const char* const npy_env = std::getenv("NPY_DISABLE_CPU_FEATURES");
  if (npy_env != nullptr) {
    return std::string_view{npy_env};
  }
  return std::string_view{};
}

// A comma-, tab-, or space-separated list of features to disable.
// https://numpy.org/devdocs/reference/simd/build-options.html#runtime-dispatch
std::vector<std::string_view> GetParsedDisableCpuFeaturesEnvVar() {
  std::vector<std::string_view> result;
  std::string_view remaining = GetDisableCpuFeaturesEnvVar();
  while (!remaining.empty()) {
    std::string_view token;
    const size_t separator = remaining.find_first_of(",\t ");
    if (separator != std::string_view::npos) {
      token = remaining.substr(0, separator);
      remaining = remaining.substr(separator + 1);
    } else {
      token = remaining;
      remaining = {};
    }
    if (!token.empty()) {
      result.push_back(token);
    }
  }
  return result;
}

struct Capabilities {
  bool allow_avx2{true};
  bool allow_avx3{true};
};

Capabilities CalcCapabilities() {
  Capabilities result;
  for (std::string_view disabled : GetParsedDisableCpuFeaturesEnvVar()) {
    if (disabled == "AVX2") {
      result.allow_avx2 = false;
      result.allow_avx3 = false;
    } else if (disabled.substr(0, 3) == "FMA") {
      result.allow_avx2 = false;
      result.allow_avx3 = false;
    } else if (disabled.substr(0, 6) == "AVX512") {
      // This is slightly conservative. The user might have only asked to turn
      // off e.g. AVX512_SPR which is not part of HWY_AVX3 anyway, but rather
      // than try to fiddle with the innumerable 512-bit vector instruction
      // capabilties matrix, we'll just shut it all down (╯°□°)╯︵ ┻━┻ .
      result.allow_avx3 = false;
    }
  }
  return result;
}

const Capabilities& GetCapabilitiesSingleton() {
  static never_destroyed<Capabilities> singleton{CalcCapabilities()};
  return singleton.access();
}

}  // namespace

int64_t GetHighwayAllowedTargetMask() {
  const Capabilities& capabilities = GetCapabilitiesSingleton();
  int64_t disallowed = 0;
  if (!capabilities.allow_avx2) {
    disallowed = disallowed | HWY_AVX2;
  }
  if (!capabilities.allow_avx3) {
    disallowed = disallowed | HWY_AVX3;
  }
  return ~disallowed;
}

}  // namespace internal
}  // namespace drake
