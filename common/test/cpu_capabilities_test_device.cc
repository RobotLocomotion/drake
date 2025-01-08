#include <iostream>

#include "hwy/detect_targets.h"

#include "drake/common/cpu_capabilities.h"

int main() {
  const int64_t allowed = drake::internal::GetHighwayAllowedTargetMask();
  std::cout << "{ "
            << "\"allow_avx2\": " << ((allowed & HWY_AVX2) ? "true" : "false")
            << ", "
            << "\"allow_avx3\": " << ((allowed & HWY_AVX3) ? "true" : "false")
            << "}" << std::endl;
  return 0;
}
