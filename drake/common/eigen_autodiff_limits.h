#pragma once

#include <limits>

namespace std {

template <typename T>
class numeric_limits<Eigen::AutoDiffScalar<T> > {
 public:
  static const bool is_specialized =
      std::numeric_limits<typename T::Scalar>::is_specialized;
  static const bool is_signed =
      std::numeric_limits<typename T::Scalar>::is_signed;
  static const bool is_integer =
      std::numeric_limits<typename T::Scalar>::is_integer;
  static const bool is_exact =
      std::numeric_limits<typename T::Scalar>::is_exact;
  static const int  radix =
      std::numeric_limits<typename T::Scalar>::radix;
  static const bool has_infinity =
      std::numeric_limits<typename T::Scalar>::has_infinity;
  static const bool has_quiet_NaN =
      std::numeric_limits<typename T::Scalar>::has_quiet_NaN;
  static const bool has_signaling_NaN =
      std::numeric_limits<typename T::Scalar>::has_signaling_NaN;
  static const bool is_iec559 =
      std::numeric_limits<typename T::Scalar>::is_iec559;
  static const bool is_bounded =
      std::numeric_limits<typename T::Scalar>::is_bounded;
  static const bool is_modulo =
      std::numeric_limits<typename T::Scalar>::is_modulo;
  static const bool traps =
      std::numeric_limits<typename T::Scalar>::traps;
  static const bool tinyness_before =
      std::numeric_limits<typename T::Scalar>::tinyness_before;
  static const std::float_denorm_style has_denorm =
      std::numeric_limits<typename T::Scalar>::has_denorm;
  static const bool has_denorm_loss =
      std::numeric_limits<typename T::Scalar>::has_denorm_loss;
  static const int min_exponent =
      std::numeric_limits<typename T::Scalar>::min_exponent;
  static const int max_exponent =
      std::numeric_limits<typename T::Scalar>::max_exponent;
  static const int min_exponent10 =
      std::numeric_limits<typename T::Scalar>::min_exponent10;
  static const int max_exponent10 =
      std::numeric_limits<typename T::Scalar>::max_exponent10;
  static const std::float_round_style round_style =
      std::numeric_limits<typename T::Scalar>::round_style;
  static const int digits = std::numeric_limits<typename T::Scalar>::digits;
  static const int digits10 =
      std::numeric_limits<typename T::Scalar>::digits10;
  static const int max_digits10 =
      std::numeric_limits<typename T::Scalar>::max_digits10;

  static constexpr typename T::Scalar min() {
    return std::numeric_limits<typename T::Scalar>::min();
  }

  static constexpr typename T::Scalar max() {
    return std::numeric_limits<typename T::Scalar>::max();
  }

  static constexpr typename T::Scalar lowest() {
    return std::numeric_limits<typename T::Scalar>::lowest();
  }

  static constexpr typename T::Scalar epsilon() {
    return std::numeric_limits<typename T::Scalar>::epsilon();
  }

  static constexpr typename T::Scalar round_error() {
    return std::numeric_limits<typename T::Scalar>::round_error();
  }

  static constexpr typename T::Scalar infinity() {
    return std::numeric_limits<typename T::Scalar>::infinity();
  }

  static constexpr typename T::Scalar quiet_NaN() {
    return std::numeric_limits<typename T::Scalar>::quiet_NaN();
  }

  static constexpr typename T::Scalar signaling_NaN() {
    return std::numeric_limits<typename T::Scalar>::signaling_NaN();
  }

  static constexpr typename T::Scalar denorm_min() {
    return std::numeric_limits<typename T::Scalar>::denorm_min();
  }
};

}  // namespace std
