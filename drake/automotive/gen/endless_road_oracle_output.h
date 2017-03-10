#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a EndlessRoadOracleOutput.
struct EndlessRoadOracleOutputIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kNetDeltaSigma = 0;
  static const int kDeltaSigmaDot = 1;
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class EndlessRoadOracleOutput : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef EndlessRoadOracleOutputIndices K;

  /// Default constructor.  Sets all rows to zero.
  EndlessRoadOracleOutput() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  EndlessRoadOracleOutput<T>* DoClone() const override {
    return new EndlessRoadOracleOutput;
  }

  /// @name Getters and Setters
  //@{
  /// net ('bumper-to-bumper') longitudinal path distance to the closest
  /// preceding vehicle/obstacle [meters]
  const T& net_delta_sigma() const {
    return this->GetAtIndex(K::kNetDeltaSigma);
  }
  void set_net_delta_sigma(const T& net_delta_sigma) {
    this->SetAtIndex(K::kNetDeltaSigma, net_delta_sigma);
  }
  /// difference in longitudinal velocity of sensing vehicle relative to the
  /// closest preceding vehicle/obstacle [m/s, +value means obstacle is slower]
  const T& delta_sigma_dot() const {
    return this->GetAtIndex(K::kDeltaSigmaDot);
  }
  void set_delta_sigma_dot(const T& delta_sigma_dot) {
    this->SetAtIndex(K::kDeltaSigmaDot, delta_sigma_dot);
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(net_delta_sigma());
    result = result && !isnan(delta_sigma_dot());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
