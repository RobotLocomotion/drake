#include "drake/common/random.h"

namespace drake {

#if  __cplusplus < 201703L
constexpr RandomGenerator::result_type RandomGenerator::default_seed;
#endif

}  // namespace drake
