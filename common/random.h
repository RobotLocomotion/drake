#pragma once

#include <random>

#include <Eigen/Core>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"

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

/**
 * Calculate the density (probability density function) of the multivariate
 * distribution.
 * @param distribution The distribution type.
 * @param x The value of the sampled vector.
 */
template <typename Derived>
typename std::enable_if<is_eigen_vector<Derived>::value,
                        typename Derived::Scalar>::type
CalcProbabilityDensity(RandomDistribution distribution,
                       const Eigen::MatrixBase<Derived>& x) {
  using T = typename Derived::Scalar;
  switch (distribution) {
    case RandomDistribution::kUniform: {
      for (int i = 0; i < x.rows(); ++i) {
        const double xi_val = ExtractDoubleOrThrow(x(i));
        if (xi_val < 0 || xi_val > 1) {
          return T(0.);
        }
      }
      return T(1.);
    }
    case RandomDistribution::kGaussian: {
      return ((-0.5 * x.array() * x.array()).exp() / std::sqrt(2 * M_PI))
          .prod();
    }
    case RandomDistribution::kExponential: {
      for (int i = 0; i < x.rows(); ++i) {
        if (ExtractDoubleOrThrow(x(i)) < 0) {
          return T(0.);
        }
      }
      return (-x.array()).exp().prod();
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace drake
