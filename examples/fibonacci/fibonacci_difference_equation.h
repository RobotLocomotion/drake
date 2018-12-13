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

@system{ FibonacciDifferenceEquation, , @output_port{Fn} }

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

See run_fibonacci.cc for the code required to output the above sequence.

*/
class FibonacciDifferenceEquation : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FibonacciDifferenceEquation)

  FibonacciDifferenceEquation() {
    // Set default initial conditions to produce the above sequence.
    DeclareDiscreteState(Eigen::Vector2d(0., 1.));

    // Update to xₙ₊₁, using a Drake "discrete update" event (occurs
    // at the beginning of step n+1).
    DeclarePeriodicDiscreteUpdate(kPeriod, 0.,  // First update is at t=0.
                                  &FibonacciDifferenceEquation::Update);

    // Present yₙ at the output port. This will be the Fibonacci element Fₙ
    // if queried at `t=n*h`.
    DeclareVectorOutputPort("Fn", systems::BasicVector<double>(1),
                            &FibonacciDifferenceEquation::Output);
  }

  /// Update period (in seconds) for the system.
  static constexpr double kPeriod = 0.25;  // Arbitrary, e.g. 0.1234 works too!

 private:
  // Update function xₙ₊₁ = f(n, xₙ).
  systems::EventStatus Update(const systems::Context<double>& context,
                              systems::DiscreteValues<double>* xd) const {
    const auto& x_n = context.get_discrete_state();
    (*xd)[0] = x_n[0] + x_n[1];
    (*xd)[1] = x_n[0];
    return systems::EventStatus::Succeeded();
  }

  // Returns the result of the output function yₙ = g(n, xₙ) when the output
  // port is evaluated at t=n*h.
  void Output(const systems::Context<double>& context,
              systems::BasicVector<double>* result) const {
    const double F_n = context.get_discrete_state()[0];  // xₙ[0]
    (*result)[0] = F_n;
  }
};

}  // namespace fibonacci
}  // namespace examples
}  // namespace drake
