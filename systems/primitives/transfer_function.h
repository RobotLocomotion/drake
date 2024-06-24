#pragma once

#include "drake/common/symbolic/rational_function.h"

namespace drake {
namespace systems {

/** Represents a linear-time-invariant (LTI) system in transfer function form,
e.g. complex rational functions corresponding to the Laplace transform or
z-transform of the impulse response
(https://en.wikipedia.org/wiki/Transfer_function).

For example, to create the transfer function H(s) = 1/(s + 1), you can write:
@code{.cc}
auto s = TransferFunction::s();
TransferFunction H(1.0 / (s + 1.0));
@endcode

To create the transfer function H(z) = 1/(z - 0.5), you can write:
@code{.cc}
auto z = TransferFunction::z();
double time_step = 0.1;
TransferFunction H(1.0 / (z - 0.5), time_step);
@endcode

Note that TransferFunction is *not* a LeafSystem. To use it in the Systems
framework, you must convert it to state-space form, represented by
LinearSystem.
*/
class TransferFunction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TransferFunction);

  /** Constructs a zero-input, zero-output (continuous-time) transfer function.
   */
  TransferFunction();

  /** Constructs a transfer function from the symbolic form.  If @p time_step
  is zero, then the model is of a continuous-time system, and @p H must only be
  a function of s().  If @p time_step > 0, then the model is of a discrete-time
  system, and @p H must only be a function of z().
  @throws std::exception if @p time_step is negative.
  @throws std::exception if @p H has variables other than s() or z(), as
  described above. */
  explicit TransferFunction(MatrixX<symbolic::RationalFunction> H,
                            double time_step = 0.0);

  /** Convenience overload for single-input, single-output (SISO) systems. */
  explicit TransferFunction(symbolic::RationalFunction H,
                            double time_step = 0.0);

  /// TODO(russt): Construct a TransferFunction from a LinearSystem once we
  /// have more symbolic support for complex numbers (#20159).

  ~TransferFunction() = default;

  /** Returns the transfer function matrix. */
  const MatrixX<symbolic::RationalFunction>& H() const { return H_; }

  /** Returns the time step. */
  double time_step() const { return time_step_; }

  /** Returns a symbolic::RationalFunction `s` which can be used to build a
  continuous-time transfer function symbolically. */
  static symbolic::RationalFunction s() {
    return symbolic::RationalFunction(symbolic::Monomial(s_var()));
  }

  /** Returns the singleton symbolic::Variable denoting the complex frequency
  (σ+jω) used in the Laplace transform for continuous-time transfer functions.
  */
  static symbolic::Variable s_var();

  /** Returns a symbolic::RationalFunction `s` which can be used to build a
  discrete-time transfer function symbolically. */
  static symbolic::RationalFunction z() {
    return symbolic::RationalFunction(symbolic::Monomial(z_var()));
  }

  /** Returns the singleton symbolic::Variable denoting the complex frequency
  (σ+jω) used in the Z transform for discrete-time transfer functions. */
  static symbolic::Variable z_var();

  // TODO(russt): Add methods to generate the various state-space realizations.

 private:
  const MatrixX<symbolic::RationalFunction> H_;
  const double time_step_{0.0};
};

}  // namespace systems
}  // namespace drake
