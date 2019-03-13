#pragma once

#include <random>

#include "drake/common/drake_copyable.h"

namespace drake {
/// Defines Drake's canonical implementation of the UniformRandomBitGenerator
/// C++ concept (as well as a few conventional extras beyond the concept, e.g.,
/// seeds).  This uses the 32-bit Mersenne Twister mt19937 by Matsumoto and
/// Nishimura, 1998.  For more information, see
/// https://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine
class RandomGenerator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RandomGenerator)

  using result_type = std::mt19937::result_type;

  RandomGenerator() = default;
  explicit RandomGenerator(result_type value) : generator_(value) {}

  static constexpr result_type min() { return std::mt19937::min(); }
  static constexpr result_type max() { return std::mt19937::max(); }
  result_type operator()() { return generator_(); }

  static constexpr result_type default_seed = std::mt19937::default_seed;

 private:
  std::mt19937 generator_{};
};

/// Drake supports explicit reasoning about a few carefully chosen random
/// distributions.
enum class RandomDistribution {
  kUniform = 0,   ///< Vector elements are independent and uniformly distributed
                  ///  ∈ [0.0, 1.0).
  kGaussian = 1,  ///< Vector elements are independent and drawn from a
                  ///  mean-zero, unit-variance normal (Gaussian) distribution.
  kExponential = 2,  ///< Vector elements are independent and drawn from an
                     ///  exponential distribution with λ=1.0.
};

}  // namespace drake
