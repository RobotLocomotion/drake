#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

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
    Fₙ₊₁ = Fₙ + Fₙ₋₁, with F₀ ≜ 0, F₁ ≜ 1.
```
Analogously to a second-order ODE, we can write this second order system as a
pair of first-order difference equations, using two state variables
`x = {x[0], x[1]}` (we're using square brackets for indexing the 2-element
vector x, _not_ for step number!). In this case xₙ[0] holds Fₙ (the value of F
at step n) while xₙ[1] holds Fₙ₋₁ (the previous value, i.e. the value of F at
step n-1). Here is the discrete system:
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
  fibonacci.Initialize(&simulator.get_mutable_context());
  simulator.StepTo(8 * FibonacciDifferenceEquation::kPeriod);
@endcode
*/
class FibonacciDifferenceEquation : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FibonacciDifferenceEquation)

  FibonacciDifferenceEquation() {
    DeclareDiscreteState(2 /* x[0], x[1] */);

    // Output yₙ using a Drake "publish" event (occurs at the end of step n).
    DeclarePeriodicEvent(kPeriod, 0.,  // period, offset
                         systems::PublishEvent<double>(
                             [this](const systems::Context<double>& context,
                                    const systems::PublishEvent<double>&) {
                               Output(context);
                             }));

    // Update to xₙ₊₁ (x_np1), using a Drake "discrete update" event (occurs
    // at the beginning of step n+1).
    DeclarePeriodicEvent(kPeriod, 0.,
                         systems::DiscreteUpdateEvent<double>(
                             [this](const systems::Context<double>& context,
                                    const systems::DiscreteUpdateEvent<double>&,
                                    systems::DiscreteValues<double>* x_np1) {
                               Update(context, x_np1);
                             }));
  }

  // Update function xₙ₊₁ = f(n, xₙ).
  void Update(const systems::Context<double>& context,
      systems::DiscreteValues<double>* x_np1) const {
    const auto& x_n = context.get_discrete_state();
    (*x_np1)[0] = x_n[0] + x_n[1];
    (*x_np1)[1] = x_n[0];
  }

  // Output function yₙ = g(n, xₙ). (Here, just writes 'n: Fₙ' to cout.)
  void Output(const systems::Context<double>& context) const {
    const double t = context.get_time();
    const int n = static_cast<int>(std::round(t / kPeriod));
    const int F_n = context.get_discrete_state()[0];  // xₙ[0]
    std::cout << n << ": " << F_n << "\n";
  }

  // Set initial conditions x₀.
  void Initialize(systems::Context<double>* context) const {
    auto& x_0 = context->get_mutable_discrete_state();
    x_0[0] = 0.;
    x_0[1] = 1.;
  }

  static constexpr double kPeriod = 1.;  // Arbitrary, e.g. 0.1234 works too!
};

}  // namespace fibonacci
}  // namespace examples
}  // namespace drake
