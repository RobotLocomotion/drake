#pragma once

#include <random>

namespace drake {
// TODO(russt): As discussed with sammy-tri, we could replace this with a
// a templated class that exposes the required methods from the concept.

// N.B. Denoting alias in docs for Python.
/// Defines the implementation of the stdc++ concept UniformRandomBitGenerator
/// to be used in Drake. This is provided as a work-around to enable the use of
/// the generator in virtual methods (which cannot be templated on the generator
/// type) of system classes.
///
/// @note This is an alias for mt19937, 32-bit Mersenne Twister by Matsumoto
/// and Nishimura, 1998. See
/// https://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine for
/// more information.
using RandomGenerator = std::mt19937;

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
