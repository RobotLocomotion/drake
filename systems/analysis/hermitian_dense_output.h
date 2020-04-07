#pragma once

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/analysis/stepwise_dense_output.h"

namespace drake {
namespace systems {

namespace internal {

/// Returns @p input_matrix as an Eigen::Matrix<double, ...> with the same size
/// allocation as @p input_matrix.  Calls ExtractDoubleOrThrow on each element
/// of the matrix, and therefore throws if any one of the extractions fail.
/// @see ExtractDoubleOrThrow(const T&)
template <typename Derived>
MatrixX<double> ExtractDoublesOrThrow(
    const Eigen::MatrixBase<Derived>& input_matrix) {
  return input_matrix.unaryExpr([] (const typename Derived::Scalar& value) {
      return ExtractDoubleOrThrow(value);
  });
}

/// Converts an STL vector of scalar type `S` elements to an STL vector
/// of double type elements, failing at runtime if the type cannot be
/// converted.
/// @see ExtractDoubleOrThrow(const T&)
/// @tparam S A valid Eigen scalar type.
template <typename S>
std::vector<double> ExtractDoublesOrThrow(const std::vector<S>& input_vector) {
  std::vector<double> output_vector{};
  output_vector.reserve(input_vector.size());
  std::transform(input_vector.begin(), input_vector.end(),
                 std::back_inserter(output_vector),
                 [] (const S& value) {
                   return ExtractDoubleOrThrow(value);
                 });
  return output_vector;
}

/// Converts an STL vector of matrices with scalar type `S` elements to an STL
/// vector of matrices with double type elements, failing at runtime if the type
/// cannot be converted.
/// @see ExtractDoublesOrThrow(const MatrixX<T>&)
/// @tparam S A valid Eigen scalar type.
template <typename S>
std::vector<MatrixX<double>>
ExtractDoublesOrThrow(const std::vector<MatrixX<S>>& input_vector) {
  std::vector<MatrixX<double>> output_vector{};
  output_vector.reserve(input_vector.size());
  std::transform(input_vector.begin(), input_vector.end(),
                 std::back_inserter(output_vector),
                 [] (const MatrixX<S>& value) {
                   return ExtractDoublesOrThrow(value);
                 });
  return output_vector;
}

}  // namespace internal

/// A StepwiseDenseOutput class implementation using Hermitian interpolators,
/// and therefore a _continuous extension_ of the solution 𝐱(t) (see
/// [Engquist, 2105]). This concept can be recast as a type of dense output that
/// is continuous.
///
/// Updates take the form of integration steps, for which state 𝐱 and state time
/// derivative d𝐱/dt are known at least at both ends of the step. Hermite cubic
/// polynomials are then constructed upon @ref StepwiseDenseOutput::Consolidate
/// "consolidation", yielding a C1 extension of the solution 𝐱(t).
///
/// Hermitian continuous extensions exhibit the same truncation error as that
/// of the integration scheme being used for up to 3rd order schemes (see
/// [Hairer, 1993]).
///
/// From a performance standpoint, memory footprint and evaluation overhead
/// (i.e. the computational cost of an evaluation) increase linearly and
/// logarithmically with the amount of steps taken, respectively.
///
/// - [Engquist, 2105] B. Engquist. Encyclopedia of Applied and Computational
///                    Mathematics, p. 339, Springer, 2015.
/// - [Hairer, 1993] E. Hairer, S. Nørsett and G. Wanner. Solving Ordinary
///                  Differential Equations I (Nonstiff Problems), p.190,
///                  Springer, 1993.
/// @tparam_default_scalar
template <typename T>
class HermitianDenseOutput final : public StepwiseDenseOutput<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HermitianDenseOutput)

  /// An integration step representation class, holding just enough
  /// for Hermitian interpolation: three (3) related sets containing
  /// step times {t₀, ..., tᵢ₋₁, tᵢ} where tᵢ ∈ ℝ, step states
  /// {𝐱₀, ..., 𝐱ᵢ₋₁, 𝐱ᵢ} where 𝐱ᵢ ∈ ℝⁿ, and state derivatives
  /// {d𝐱/dt₀, ..., d𝐱/dtᵢ₋₁, d𝐱/dtᵢ} where d𝐱/dtᵢ ∈ ℝⁿ.
  ///
  /// This step definition allows for intermediate time, state and state
  /// derivative triplets (e.g. the integrator internal stages) to improve
  /// interpolation.
  ///
  /// @note The use of column matrices instead of plain vectors helps reduce
  ///       HermitianDenseOutput construction overhead, as this type of dense
  ///       output leverages a PiecewisePolynomial instance that takes matrices.
  class IntegrationStep {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IntegrationStep)
    /// Constructs an empty step.
    IntegrationStep() = default;

    /// Constructs a zero length step (i.e. a step containing a single time,
    /// state and state derivative triplet) from column matrices.
    ///
    /// @param initial_time Initial time t₀ where the step starts.
    /// @param initial_state Initial state vector 𝐱₀ at @p initial_time
    ///                      as a column matrix.
    /// @param initial_state_derivative Initial state derivative vector
    ///                                 d𝐱/dt₀ at @p initial_time as a
    ///                                 column matrix.
    /// @throws std::runtime_error
    ///   if given @p initial_state 𝐱₀ is not a column matrix.<br>
    ///   if given @p initial_state_derivative d𝐱/t₀ is not a column
    ///   matrix.<br>
    ///   if given @p initial_state 𝐱₀ and @p initial_state_derivative
    ///   d𝐱/dt₀ do not match each other's dimension.
    IntegrationStep(const T& initial_time, MatrixX<T> initial_state,
                    MatrixX<T> initial_state_derivative) {
      ValidateStepExtendTripletOrThrow(initial_time, initial_state,
                                       initial_state_derivative);
      times_.push_back(initial_time);
      states_.push_back(std::move(initial_state));
      state_derivatives_.push_back(std::move(initial_state_derivative));
    }

    /// Extends the step forward in time from column matrices.
    ///
    /// Provided @p time, @p state and @p state_derivative are appended
    /// to the current step, effectively increasing its time length.
    ///
    /// @param time Time tᵢ to extend the step to.
    /// @param state State vector 𝐱ᵢ at @p time tᵢ as a column matrix.
    /// @param state_derivative State derivative vector d𝐱/dtᵢ at @p time tᵢ
    ///                         as a column matrix.
    /// @throws std::runtime_error
    ///   if given @p state 𝐱ᵢ is not a column matrix.<br>
    ///   if given @p state_derivative d𝐱/dtᵢ is not a column matrix.<br>
    ///   if given @p time tᵢ is not greater than the previous time
    ///   tᵢ₋₁ in the step.<br>
    ///   if given @p state 𝐱ᵢ dimension does not match the dimension of
    ///   the previous state 𝐱ᵢ₋₁.<br>
    ///   if given @p state 𝐱ᵢ and @p state_derivative d𝐱/dtᵢ do not
    ///   match each other's dimension.
    void Extend(const T& time, MatrixX<T> state, MatrixX<T> state_derivative) {
      ValidateStepExtendTripletOrThrow(time, state, state_derivative);
      times_.push_back(time);
      states_.push_back(std::move(state));
      state_derivatives_.push_back(std::move(state_derivative));
    }

    /// Returns step start time t₀ (that of the first time, state and state
    /// derivative triplet), which may coincide with its end time tᵢ (that of
    /// the last time, state and state derivative triplet) if the step has zero
    /// length (that is, it contains a single triplet).
    const T& start_time() const { return times_.front(); }

    /// Returns step end time tᵢ (that of the first time, state and state
    /// derivative triplet), which may coincide with its start time t₀ (that of
    /// the last time, state and state derivative triplet) if the step has zero
    /// length (that is, it contains a single triplet).
    const T& end_time() const { return times_.back(); }

    /// Returns the step state 𝐱 size (i.e. dimension).
    int size() const {
      return states_.back().rows();
    }

    /// Returns step times {t₀, ..., tᵢ₋₁, tᵢ}.
    const std::vector<T>& get_times() const { return times_; }

    /// Returns step states {𝐱₀, ..., 𝐱ᵢ₋₁, 𝐱ᵢ} as column matrices.
    const std::vector<MatrixX<T>>& get_states() const { return states_; }

    /// Gets step state derivatives {d𝐱/dt₀, ..., d𝐱/dtᵢ₋₁, d𝐱/dtᵢ}
    /// as column matrices.
    const std::vector<MatrixX<T>>& get_state_derivatives() const {
      return state_derivatives_;
    }

   private:
    // Validates step update triplet for consistency between the triplet
    // and with current step content.
    //
    // @see Extend(const T&, MatrixX<T>, MatrixX<T>)
    void ValidateStepExtendTripletOrThrow(
        const T& time, const MatrixX<T>& state,
        const MatrixX<T>& state_derivative) {
      if (state.cols() != 1) {
        throw std::runtime_error("Provided state for step is "
                                 "not a column matrix.");
      }
      if (state_derivative.cols() != 1) {
        throw std::runtime_error("Provided state derivative for "
                                 " step is not a column matrix.");
      }
      if (!times_.empty()) {
        if (time < times_.front()) {
          throw std::runtime_error("Step cannot be extended"
                                   " backwards in time.");
        }
        if (time <= times_.back()) {
          throw std::runtime_error("Step already extends up"
                                   " to the given time.");
        }
      }
      if (!states_.empty() && states_.back().rows() != state.rows()) {
          throw std::runtime_error("Provided state dimensions do not "
                                   "match that of the states in the step.");
      }
      if (state.rows() != state_derivative.rows()) {
        throw std::runtime_error("Provided state and state derivative "
                                 "dimensions do not match.");
      }
    }
    // Step times, ordered in increasing order.
    std::vector<T> times_{};
    // Step states, ordered as to match its corresponding time in `times_`.
    std::vector<MatrixX<T>> states_{};
    // Step state derivatives, ordered as to match its corresponding
    // time in `times_`.
    std::vector<MatrixX<T>> state_derivatives_{};
  };

  HermitianDenseOutput() = default;

  /// Initialize the DenseOutput with an existing trajectory.
  explicit HermitianDenseOutput(
      const trajectories::PiecewisePolynomial<T>& trajectory)
      : start_time_(trajectory.start_time()),
        end_time_(trajectory.end_time()) {
    if constexpr (std::is_same<T, double>::value) {
      continuous_trajectory_ = trajectory;
      return;
    }

    // Create continuous_trajectory_ by converting all the segments to double.
    using trajectories::PiecewisePolynomial;
    const std::vector<T>& breaks = trajectory.get_segment_times();
    for (int i = 0; i < trajectory.get_number_of_segments(); i++) {
      const typename PiecewisePolynomial<T>::PolynomialMatrix& poly =
          trajectory.getPolynomialMatrix(i);
      MatrixX<Polynomiald> polyd = poly.unaryExpr([](const Polynomial<T>& p) {
        return Polynomiald(
            internal::ExtractDoublesOrThrow(p.GetCoefficients()));
      });
      continuous_trajectory_.ConcatenateInTime(
          PiecewisePolynomial<double>({polyd},
                                      {ExtractDoubleOrThrow(breaks[i]),
                                       ExtractDoubleOrThrow(breaks[i + 1])}));
    }
  }

  /// Update output with the given @p step.
  ///
  /// Provided @p step is queued for later consolidation. Note that
  /// the time the @p step extends cannot be readily evaluated (see
  /// StepwiseDenseOutput class documentation).
  ///
  /// @param step Integration step to update this output with.
  /// @throws std::runtime_error
  ///   if given @p step has zero length.<br>
  ///   if given @p step does not ensure C1 continuity at the end of
  ///   this dense output.<br>
  ///   if given @p step dimensions does not match this dense output
  ///   dimensions.
  void Update(IntegrationStep step) {
    ValidateStepCanBeConsolidatedOrThrow(step);
    raw_steps_.push_back(std::move(step));
  }

  void Rollback() override {
    if (raw_steps_.empty()) {
      throw std::logic_error("No updates to rollback.");
    }
    raw_steps_.pop_back();
  }

  void Consolidate() override {
    if (raw_steps_.empty()) {
      throw std::logic_error("No updates to consolidate.");
    }
    for (const IntegrationStep& step : raw_steps_) {
      continuous_trajectory_.ConcatenateInTime(
          trajectories::PiecewisePolynomial<double>::CubicHermite(
              internal::ExtractDoublesOrThrow(step.get_times()),
              internal::ExtractDoublesOrThrow(step.get_states()),
              internal::ExtractDoublesOrThrow(step.get_state_derivatives())));
    }
    start_time_ = continuous_trajectory_.start_time();
    end_time_ = continuous_trajectory_.end_time();
    last_consolidated_step_ = std::move(raw_steps_.back());
    raw_steps_.clear();
  }

 protected:
  VectorX<T> DoEvaluate(const T& t) const override {
    const MatrixX<double> matrix_value =
        continuous_trajectory_.value(ExtractDoubleOrThrow(t));
    return matrix_value.col(0).cast<T>();
  }

  T DoEvaluateNth(const T& t, const int n) const override {
    return continuous_trajectory_.scalarValue(
        ExtractDoubleOrThrow(t), n, 0);
  }

  bool do_is_empty() const override {
    return continuous_trajectory_.empty();
  }

  int do_size() const override {
    return continuous_trajectory_.rows();
  }

  const T& do_end_time() const override { return end_time_; }

  const T& do_start_time() const override { return start_time_; }

 private:
  // Validates that the provided @p step can be consolidated into this
  // dense output.
  // @see Update(const IntegrationStep&)
  void ValidateStepCanBeConsolidatedOrThrow(const IntegrationStep& step) {
    if (step.start_time() == step.end_time()) {
      throw std::runtime_error("Provided step has zero length "
                               "i.e. start time and end time "
                               "are equal.");
    }
    if (!raw_steps_.empty()) {
      EnsureOutputConsistencyOrThrow(step, raw_steps_.back());
    } else if (!continuous_trajectory_.empty()) {
      EnsureOutputConsistencyOrThrow(step, last_consolidated_step_);
    }
  }

  // Ensures that this dense output would remain consistent if the
  // provided @p step were to be consolidated at its end.
  // @param next_step Integration step to be taken.
  // @param prev_step Last integration step consolidated or to be
  //                  consolidated into dense output.
  // @throws std::runtime_error
  //   if given @p next_step does not ensure C1 continuity at the
  //   end of the given @p prev_step.<br>
  //   if given @p next_step dimensions does not match @p prev_step
  //   dimensions.
  static void EnsureOutputConsistencyOrThrow(const IntegrationStep& next_step,
                                             const IntegrationStep& prev_step) {
    using std::abs;
    using std::max;

    if (prev_step.size() != next_step.size()) {
      throw std::runtime_error("Provided step dimensions and previous"
                               " step dimensions do not match.");
    }
    // Maximum time misalignment between previous step and next step that
    // can still be disregarded as a discontinuity in time.
    const T& prev_end_time = prev_step.end_time();
    const T& next_start_time = next_step.start_time();
    const T allowed_time_misalignment =
        max(abs(prev_end_time), T{1.}) * std::numeric_limits<T>::epsilon();
    const T time_misalignment = abs(prev_end_time - next_start_time);
    if (time_misalignment > allowed_time_misalignment) {
      throw std::runtime_error("Provided step start time and"
                               " previous step end time differ.");
    }
    // We can't sanity check the state values when using symbolic expressions.
    if constexpr (scalar_predicate<T>::is_bool) {
      const MatrixX<T>& prev_end_state = prev_step.get_states().back();
      const MatrixX<T>& next_start_state = next_step.get_states().front();
      if (!prev_end_state.isApprox(next_start_state)) {
        throw std::runtime_error(
            "Provided step start state and previous step end state differ. "
            "Cannot ensure C0 continuity.");
      }
      const MatrixX<T>& prev_end_state_derivative =
          prev_step.get_state_derivatives().back();
      const MatrixX<T>& next_start_state_derivative =
          next_step.get_state_derivatives().front();
      if (!prev_end_state_derivative.isApprox(next_start_state_derivative)) {
        throw std::runtime_error(
            "Provided step start state derivative and previous step end state "
            "derivative differ. Cannot ensure C1 continuity.");
      }
    }
  }

  // TODO(hidmic): Remove redundant time-keeping member fields when
  // PiecewisePolynomial supports return by-reference of its time extents.
  // It currently returns them by-value, double type only, and thus the
  // need for this storage in order to meet DenseOutput::start_time()
  // and DenseOutput::end_time() API.

  // The smallest time at which the output is defined.
  T start_time_{};
  // The largest time at which the output is defined.
  T end_time_{};

  // The last integration step consolidated into `continuous_trajectory_`,
  // useful to validate the next integration steps.
  // @see EnsureOutputConsistencyOrThrow
  IntegrationStep last_consolidated_step_{};

  // The integration steps taken but not consolidated yet (via Consolidate()).
  std::vector<IntegrationStep> raw_steps_{};

  // TODO(hidmic): When PiecewisePolynomial supports scalar types other than
  // doubles, pass in the template parameter T to it too and remove all scalar
  // type conversions.  UPDATE(russt): New plan is to deprecate this class, as
  // PiecewisePolynomial can serve the intended role by itself.

  // The underlying PiecewisePolynomial continuous trajectory.
  trajectories::PiecewisePolynomial<double> continuous_trajectory_{};
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::HermitianDenseOutput)
