#include "drake/systems/primitives/trajectory_source.h"

#include <limits>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

using trajectories::Trajectory;

template <typename T>
TrajectorySource<T>::TrajectorySource(const Trajectory<T>& trajectory,
                                      int output_derivative_order,
                                      bool zero_derivatives_beyond_limits)
    : SingleOutputVectorSource<T>(
          SystemTypeTag<TrajectorySource>{},
          trajectory.rows() * (1 + output_derivative_order)),
      // Make a copy of the input trajectory.
      trajectory_(trajectory.Clone()),
      clamp_derivatives_(zero_derivatives_beyond_limits) {
  // This class does not currently support trajectories which output
  // more complicated matrices.
  DRAKE_THROW_UNLESS(trajectory.cols() == 1);
  DRAKE_THROW_UNLESS(output_derivative_order >= 0);

  for (int i = 0; i < output_derivative_order; ++i) {
    if (i == 0) {
      derivatives_.push_back(trajectory_->MakeDerivative());
    } else {
      derivatives_.push_back(derivatives_[i - 1]->MakeDerivative());
    }
  }

  CheckInvariants();
}

template <typename T>
template <typename U>
TrajectorySource<T>::TrajectorySource(const TrajectorySource<U>& other)
    : SingleOutputVectorSource<T>(SystemTypeTag<TrajectorySource>{},
                                  other.get_output_port().size()),
      clamp_derivatives_{other.clamp_derivatives_} {
  other.CheckInvariants();
  // The scalar_conversion::Traits in our header only allows two possible
  // conversion patterns (the "if" and "else" cases here).
  if constexpr (std::is_same_v<U, double>) {
    // We're converting from double to AutoDiffXd or Expression. We'll keep a
    // copy of the Trajectory<double> as a "failsafe", until the user replaces
    // it by calling UpdateTrajectory().
    failsafe_trajectory_ = other.trajectory_->Clone();
    for (auto& other_derivative : other.derivatives_) {
      failsafe_derivatives_.emplace_back(other_derivative->Clone());
    }
  } else {
    // We're converting from AutoDiffXd to double, probably for a Clone().
    // This is allowed iff `other` was still using the failsafe trajectory.
    static_assert(std::is_same_v<U, AutoDiffXd>);
    static_assert(std::is_same_v<T, double>);
    if (other.failsafe_trajectory_ == nullptr) {
      throw std::logic_error(fmt::format(
          "System {} of type {} does not support scalar conversion to type {}",
          this->GetSystemPathname(), "AutoDiffXd", "double"));
    }
    trajectory_ = other.failsafe_trajectory_->Clone();
    for (auto& other_derivative : other.failsafe_derivatives_) {
      derivatives_.emplace_back(other_derivative->Clone());
    }
  }
  CheckInvariants();
}

template <typename T>
TrajectorySource<T>::~TrajectorySource() = default;

template <typename T>
void TrajectorySource<T>::CheckInvariants() const {
  const bool is_normal = (trajectory_ != nullptr);
  const bool is_failsafe = (failsafe_trajectory_ != nullptr);
  if constexpr (std::is_same_v<T, double>) {
    DRAKE_DEMAND(is_normal);
  }
  if (is_normal) {
    DRAKE_DEMAND(!is_failsafe);
    DRAKE_DEMAND(failsafe_derivatives_.empty());
  } else {
    DRAKE_DEMAND(is_failsafe);
    DRAKE_DEMAND(derivatives_.empty());
  }
}

template <typename T>
void TrajectorySource<T>::UpdateTrajectory(
    const trajectories::Trajectory<T>& trajectory) {
  CheckInvariants();
  const int rows = (trajectory_ != nullptr) ? trajectory_->rows()
                                            : failsafe_trajectory_->rows();
  DRAKE_THROW_UNLESS(trajectory.rows() == rows);
  DRAKE_THROW_UNLESS(trajectory.cols() == 1);

  trajectory_ = trajectory.Clone();
  for (int i = 0; i < ssize(derivatives_); ++i) {
    if (i == 0) {
      derivatives_[i] = trajectory_->MakeDerivative();
    } else {
      derivatives_[i] = derivatives_[i - 1]->MakeDerivative();
    }
  }

  failsafe_trajectory_ = nullptr;
  failsafe_derivatives_.clear();
  CheckInvariants();
}

namespace {
// Given a non-double `time`, either casts it to a `double` without discarding
// any information, or else throws an exception.
template <typename T>
double ExtractFailsafeTime(const T& time) {
  if constexpr (std::is_same_v<T, AutoDiffXd>) {
    if (!time.derivatives().isZero(0.0)) {
      throw std::logic_error(
          "Cannot take gradients w.r.t. time with this TrajectorySource. "
          "You must call UpdateTrajectory() to provide a "
          "Trajectory<AutoDiffXd>.");
    }
    return ExtractDoubleOrThrow(time);
  }
  if constexpr (std::is_same_v<T, symbolic::Expression>) {
    if (!is_constant(time)) {
      throw std::logic_error(
          "Cannot output using symbolic time with this TrajectorySource. "
          "You must call UpdateTrajectory() to provide a "
          "Trajectory<Expression>.");
    }
    return ExtractDoubleOrThrow(time);
  }
  DRAKE_UNREACHABLE();
}

// Implements DoCalcVectorOutput, after we've decided whether to use the
// trajectory_ or fallback_trajectory_.
//
// @tparam T1 the scalar type of the Trajectory being evaluated
// @tparam T2 the scalar type of the source's output
template <typename T1, typename T2>
void CalcVectorOutputImpl(
    const T1& time, const std::unique_ptr<Trajectory<T1>>& trajectory,
    const bool clamp_derivatives,
    const std::vector<std::unique_ptr<Trajectory<T1>>>& derivatives,
    Eigen::VectorBlock<T2>* output) {
  // Output the value.
  const int len = trajectory->rows();
  output->head(len) = trajectory->value(time);
  // Output the derivatives (possibly subject to clamping).
  boolean<T1> want_derivatives =
      !boolean<T1>(clamp_derivatives) ||
      (time >= trajectory->start_time() && time <= trajectory->end_time());
  if constexpr (scalar_predicate<T1>::is_bool) {
    // When the trajectory is not symbolic, we can use block-wise evaluation.
    if (want_derivatives) {
      for (int i = 0; i < ssize(derivatives); ++i) {
        output->segment(len * (i + 1), len) = derivatives[i]->value(time);
      }
    } else {
      output->segment(len, len * ssize(derivatives)).setZero();
    }
  } else {
    // When the trajectory is symbolic, we must operate one Expression at a time
    // because there is no block-wise form of if_then_else.
    for (int i = 0; i < ssize(derivatives); ++i) {
      const VectorX<T1> value = derivatives[i]->value(time);
      for (int j = 0; j < len; ++j) {
        (*output)[len * (i + 1) + j] =
            if_then_else(want_derivatives, value[j], 0.0);
      }
    }
  }
}
}  // namespace

template <typename T>
void TrajectorySource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  CheckInvariants();
  const T time = context.get_time();
  if (failsafe_trajectory_ != nullptr) {
    const double failsafe_time = ExtractFailsafeTime(time);
    CalcVectorOutputImpl(failsafe_time, failsafe_trajectory_,
                         clamp_derivatives_, failsafe_derivatives_, output);
  } else {
    CalcVectorOutputImpl(time, trajectory_, clamp_derivatives_, derivatives_,
                         output);
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::TrajectorySource);
