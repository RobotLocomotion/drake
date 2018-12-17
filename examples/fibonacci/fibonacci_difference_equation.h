#pragma once

#include <cmath>
#include <iostream>

#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace fibonacci {

/** A pure discrete system that generates the Fibonacci sequence Fₙ using
a difference equation.

In general, a discrete system has a difference equation (update function),
output function, and (for simulation) an initial value:
- _update_ function `xₙ₊₁ = f(n, xₙ, uₙ)`, and
- _output_ function `yₙ = g(n, xₙ, uₙ)`, and
- `x₀ ≜ xᵢₙᵢₜ`.

where x is a vector of discrete variables, u is a vector of external inputs,
and y is a vector of values that constitute the desired output of the discrete
system. The subscript indicates the value of these variables at step n, where
n is an integer 0, 1, 2, ... .

The Fibonacci sequence is defined by the second-order difference equation
```
    Fₙ₊₁ = Fₙ + Fₙ₋₁, with F₀ ≜ 0, F₁ ≜ 1,
```
which uses no input.

We can write this second order system as a pair of first-order difference
equations, using two state variables `x = {x[0], x[1]}` (we're using square
brackets for indexing the 2-element vector x, _not_ for step number!). In this
case xₙ[0] holds Fₙ (the value of F at step n) while xₙ[1] holds Fₙ₋₁ (the
previous value, i.e. the value of F at step n-1). Here is the discrete system:
```
    xₙ₊₁ = {xₙ[0] + xₙ[1], xₙ[0]}   // f()
      yₙ = xₙ[0]                    // g()
      x₀ ≜ {0, 1}                   // xᵢₙᵢₜ
```

We want to show how to emulate this difference equation in Drake's hybrid
simulator, which advances a continuous time variable t rather than a discrete
step number n. To do that, we pick an arbitrary discrete period h, and show that
publishing at `t = n*h` produces the expected result
```
    n  0  1  2  3  4  5  6  7  8
    Fₙ 0  1  1  2  3  5  8 13 21 ...
```

The code required to produce the above sequence (see run_fibonacci.cc) is just
@code{.cpp}
  FibonacciDifferenceEquation fibonacci;
  systems::Simulator<double> simulator(fibonacci);
  simulator.StepTo(8 * FibonacciDifferenceEquation::kPeriod);
@endcode
*/
// TODO(sherm1) Simplify the periodic event specifications here when
// PR #10132 lands.
class FibonacciDifferenceEquation : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FibonacciDifferenceEquation)

  FibonacciDifferenceEquation() {
    // Set default initial conditions to produce the above sequence.
    DeclareDiscreteState(Eigen::Vector2d(0., 1.));

    // Output yₙ using a Drake "publish" event (occurs at the end of step n).
    DeclarePeriodicEvent(kPeriod, 0.,  // period, offset
                         systems::PublishEvent<double>(
                             [this](const systems::Context<double>& context,
                                    const systems::PublishEvent<double>&) {
                               PrintResult(context);
                             }));

    // Update to xₙ₊₁, using a Drake "discrete update" event (occurs
    // at the beginning of step n+1).
    DeclarePeriodicEvent(kPeriod, 0.,
                         systems::DiscreteUpdateEvent<double>(
                             [this](const systems::Context<double>& context,
                                    const systems::DiscreteUpdateEvent<double>&,
                                    systems::DiscreteValues<double>* xd) {
                               Update(context, xd);
                             }));
  }

  /// Update period (in seconds) for the system.
  static constexpr double kPeriod = 0.7;  // Arbitrary, e.g. 0.1234 works too!

 private:
  // Update function xₙ₊₁ = f(n, xₙ).
  void Update(const systems::Context<double>& context,
                              systems::DiscreteValues<double>* xd) const {
    const auto& x_n = context.get_discrete_state();
    (*xd)[0] = x_n[0] + x_n[1];
    (*xd)[1] = x_n[0];
  }

  // Print the result of the output function yₙ = g(n, xₙ) to cout.
  void PrintResult(const systems::Context<double>& context) const {
    const double t = context.get_time();
    // Because of finite floating point precision, t / kPeriod will yield the
    // desired step (n) plus or minus some error. For example, (3 * 0.7) / 0.7
    // in double precision evalutes to 2.9999999999999996, but we want int n = 3
    // (rounded), not n = 2 (truncated). Using round() first ensures that
    // casting to an int gives the correct step number.
    const int n = static_cast<int>(std::round(t / kPeriod));
    const int F_n = context.get_discrete_state()[0];  // xₙ[0]
    std::cout << n << ": " << F_n << "\n";
  }
};

}  // namespace fibonacci
}  // namespace examples
}  // namespace drake
