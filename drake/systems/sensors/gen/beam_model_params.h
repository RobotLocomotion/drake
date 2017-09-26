#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/never_destroyed.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace sensors {

/// Describes the row indices of a BeamModelParams.
struct BeamModelParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 5;

  // The index of each individual coordinate.
  static const int kLambdaShort = 0;
  static const int kSigmaHit = 1;
  static const int kProbabilityShort = 2;
  static const int kProbabilityMiss = 3;
  static const int kProbabilityUniform = 4;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `BeamModelParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BeamModelParams : public systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BeamModelParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c lambda_short defaults to 1.0 dimensionless.
  /// @arg @c sigma_hit defaults to 0.0 m.
  /// @arg @c probability_short defaults to 0.0 dimensionless.
  /// @arg @c probability_miss defaults to 0.0 dimensionless.
  /// @arg @c probability_uniform defaults to 0.0 dimensionless.
  BeamModelParams() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_lambda_short(1.0);
    this->set_sigma_hit(0.0);
    this->set_probability_short(0.0);
    this->set_probability_miss(0.0);
    this->set_probability_uniform(0.0);
  }

  BeamModelParams<T>* DoClone() const override { return new BeamModelParams; }

  /// @name Getters and Setters
  //@{
  /// The rate parameter of the (truncated) exponential distribution governing
  /// short returns
  /// @note @c lambda_short is expressed in units of dimensionless.
  /// @note @c lambda_short has a limited domain of [0.0, +Inf].
  const T& lambda_short() const { return this->GetAtIndex(K::kLambdaShort); }
  void set_lambda_short(const T& lambda_short) {
    this->SetAtIndex(K::kLambdaShort, lambda_short);
  }
  /// The standard deviation of the (truncated) Gaussian distribution governing
  /// the noisy returns of the true depth (aka hit)
  /// @note @c sigma_hit is expressed in units of m.
  /// @note @c sigma_hit has a limited domain of [0.0, +Inf].
  const T& sigma_hit() const { return this->GetAtIndex(K::kSigmaHit); }
  void set_sigma_hit(const T& sigma_hit) {
    this->SetAtIndex(K::kSigmaHit, sigma_hit);
  }
  /// The total probability of getting a short return is probability_short *
  /// p(lambda_short*w_short <= input_depth)
  /// @note @c probability_short is expressed in units of dimensionless.
  /// @note @c probability_short has a limited domain of [0.0, 1.0].
  const T& probability_short() const {
    return this->GetAtIndex(K::kProbabilityShort);
  }
  void set_probability_short(const T& probability_short) {
    this->SetAtIndex(K::kProbabilityShort, probability_short);
  }
  /// The probability of ignoring the input depth and simply returning the max
  /// range of the sensor
  /// @note @c probability_miss is expressed in units of dimensionless.
  /// @note @c probability_miss has a limited domain of [0.0, 1.0].
  const T& probability_miss() const {
    return this->GetAtIndex(K::kProbabilityMiss);
  }
  void set_probability_miss(const T& probability_miss) {
    this->SetAtIndex(K::kProbabilityMiss, probability_miss);
  }
  /// The probability of ignoring the input depth and simple returning a uniform
  /// random value between 0 and the max range of the sensor
  /// @note @c probability_uniform is expressed in units of dimensionless.
  /// @note @c probability_uniform has a limited domain of [0.0, 1.0].
  const T& probability_uniform() const {
    return this->GetAtIndex(K::kProbabilityUniform);
  }
  void set_probability_uniform(const T& probability_uniform) {
    this->SetAtIndex(K::kProbabilityUniform, probability_uniform);
  }
  //@}

  /// See BeamModelParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return BeamModelParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  decltype(T() < T()) IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(lambda_short());
    result = result && (lambda_short() >= T(0.0));
    result = result && !isnan(sigma_hit());
    result = result && (sigma_hit() >= T(0.0));
    result = result && !isnan(probability_short());
    result = result && (probability_short() >= T(0.0));
    result = result && (probability_short() <= T(1.0));
    result = result && !isnan(probability_miss());
    result = result && (probability_miss() >= T(0.0));
    result = result && (probability_miss() <= T(1.0));
    result = result && !isnan(probability_uniform());
    result = result && (probability_uniform() >= T(0.0));
    result = result && (probability_uniform() <= T(1.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(VectorX<T>* value) const override {
    value->resize(8);
    (*value)[0] = lambda_short() - T(0.0);
    (*value)[1] = sigma_hit() - T(0.0);
    (*value)[2] = probability_short() - T(0.0);
    (*value)[3] = T(1.0) - probability_short();
    (*value)[4] = probability_miss() - T(0.0);
    (*value)[5] = T(1.0) - probability_miss();
    (*value)[6] = probability_uniform() - T(0.0);
    (*value)[7] = T(1.0) - probability_uniform();
  }
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
