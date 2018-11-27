
namespace drake {
namespace systems {

/** @addtogroup discrete_systems
@brief This page describes discrete systems modeled by difference equations
(contrast to continuous systems modeled by ordinary differential equations)
as well as considerations for implementing these systems in Drake.

The state space dynamics of a discrete system is:
```
    xₙ₊₁ = f(p; n, xₙ, uₙ)    // update
    yₙ   = g(p; n, xₙ, uₙ)    // output
    x₀   = xᵢₙᵢₜ              // initialize
```

where n ∈ ℤ is the variable iterate (typically starting at zero), x ∈ ℝ is
the discrete state variable ("discrete" refers to the countability of the
elements of the sequence, x₀, x₁, ..., xₙ and not the values that x can take),
y is the desired output, u is the input, and p are the (constant)
parameters. f(.) and g(.) are the _update_ and _output_ functions,
respectively. Any of these quantities can be vector-valued. The subscript
notation (e.g., x₀) is used to show that the state, input, and output result
from a discrete process. We use bracket notation, e.g. x[1] to designate
particular elements of a vector-valued quantity (indexing from 0). Combined,
x₁[3] would be the value of the fourth element of the x vector, evaluated at
step n=1. In code we use a Latex-like underscore to indicate the step number,
so we write x_1[3] to represent x₁[3].

@image html framework/images/discrete.png "Figure 1: Plots of state (x) and output (y) of a discrete system. This plot shows one possible relationship between iterate (n) and time (t)."

The following class implements the simple discrete system
```
    xₙ₊₁ = xₙ + 1
    yₙ   = xₙ + 1
```
as a Drake LeafSystem:

@code
class SimpleDiscreteSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleDiscreteSystem)
  SimpleDiscreteSystem() {
    const int state_dim = 1;
    this->DeclarePeriodicDiscreteUpdate(kPeriod, kOffset);
    this->DeclareVectorOutputPort("y", BasicVector<double>(1),
        &SimpleDiscreteSystem::CalcOutput);
    this->DeclareDiscreteState(state_dim);
  }

  double GetX(const Context<double>& context) {
    return context.get_discrete_state()[0];
  }

 private:
  void DoCalcDiscreteVariableUpdates(
      const Context<double>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>&,
      DiscreteValues<double>* x_next) const override {
    (*x_next)[0] = GetX(context) + 1;
  }

  void DoPublish(const Context<double>& context,
      const std::vector<const PublishEvent<double>*>&) const override {
    std::cout << "Published " << GetX(context) << " at t=" <<
        context.get_time() << std::endl;
  }

  void CalcOutput(
      const Context<double>& context, BasicVector<double>* output) const {
    *output[0] = GetX(context) + 1;
  }

  const double kPeriod = 1.0;  // Update every second (h=1).
  const double kOffset = 1.0;  // First update is at one second.
};
@endcode

Stepping this system forward using the following code fragment:
@code
SimpleDiscreteSystem system;
Simulator<double> simulator(system);
simulator.set_publish_on_initialization(false);
simulator.StepTo(3.0);
@endcode
yields the following output:
@verbatim
Published 0 at t=1.0
Published 1.0 at t=2.0
Published 2.0 at t=3.0
@endverbatim

<h2>Interaction of discrete and continuous models: how Drake models discrete
 systems in a hybrid systems framework (ADVANCED)</h2>
Drake can model continuous systems, discrete systems, and hybrid
discrete/continuous systems. This section will primary focus upon a particular
design choice for implementing discrete systems in Drake's' hybrid system
framework.

First, note that discrete systems in Drake possess one or more discrete
variables (DiscreteValues) and are usually updated at a specified periodicity
(which we will denote `h`) as time advances. Observe that `h = 0.1` for the
system shown in Figure 1.

The plot below shows the state evolution of this system over time. Since Drake
allows continuous and discrete systems to interact, the discrete systems must
be sample-able at times that correspond to non-integer multiples of `h`.
Contrast how the values of `x` and `y` are held for finite time in Figure 2
with the instantaneous values of `x` and `y` in Figure 1. The labels will be
described in the next section (below).

@image html framework/images/simple-discrete.png "Figure 2: Plots of state for the discrete system x[n+1] = x[n] + 1, y[n] = x[n] + 1 (from x[0] = 0 and using h = 1). The colors and stroke (solid=state, dashed=output) correspond to the evolution of the system with first-update time at zero (black) and h (blue)."

Using time in place of iteration count allows Drake to
model the evolution of hybrid continuous/discrete systems using a single
independent variable (time). But this decision gives rise to particular
implications we will now describe.

<h3>(a) Discrete variables take on two values at their update times</h3>
The discrete system represented in our pedagogical example takes on two
values at `t = 1.0`: `x[0]` (pre-update `x`) and `x[1]` (post-update `x`).
We distinguish between pre-update and post-update times using `t⁻` and `t⁺`,
respectively. Likewise, we can distinguish between the value of `x` at pre-
and post-update times as `x(1.0⁻) = x[0]` and `x(1.0⁺) = x[1]`. We will
make use of the same notation for `u(t)`. You can think of tᵢ⁻ as the time at
the end of the iᵗʰ time step, while you can think of tᵢ⁺ as the time at the
beginning of time step i+1.

Note well that the parenthesis notation refers to the
value of the variable at a _time_ while the bracket notation refers to the value
of the variable at an _iteration_.

<h3>(b) Should the first update be at time zero?</h3>
The discrete system `x[n+1] = x[n] + 1` can be modeled two different ways
(resulting in slightly different evolutions) in Drake: the first update can
happen at time zero or it can happen at time `h`. The blue and black lines in
Figure 2 illustrate the effect of applying each strategy. The output at the
end of the last section corresponds to the black line; we reproduce it below:
@verbatim
Published 0 at t=1.0
Published 1.0 at t=2.0
Published 2.0 at t=3.0
@endverbatim

In contrast, the output corresponding to the blue line would be:
@verbatim
Published 1.0 at t=1.0
Published 2.0 at t=2.0
Published 3.0 at t=3.0
@endverbatim

In order to understand the full implications of the choice of update
time, we must examine how discrete and continuous systems interact.
Consider the hybrid continuous-discrete system `x[n+1] = x[n] + u(t)` where
`h = 1`, `x[0] = 0`, and `u(t) = t + 1`. Applying the two strategies to
initial conditions `x[0] = x(0⁻) = 0` (i.e., advancing the system through
time using Drake's Simulator) yields the plot shown in Figure 3.

@image html framework/images/simple-hybrid.png "Figure 3: Plots of state for the hybrid system x[n+1] = x[n] + u(t) (from x[0] = 0 and using h = 1 and u(t) = t + 1)."
(See the class documentation for Simulator for a detailed understanding of the stepping process.)

Examining the two pedagogical examples, the following characteristics
should become apparent.

_For the case of updates starting at time `h`_ (black line):
1. The discrete value is only advanced to `x[1]` when `t=h`.
2. The first discrete update samples the input at `h`, i.e., x[1] = `x(h⁺)` is
   computed using `u(h)`.

_For the case of updates starting at time zero_ (blue line):
1. Advancing time and state to that at `t = 1e-16` results in the discrete
   value being advanced to `x[1]`.
2. The first discrete update samples the input at `0`, i.e., x[1] = x(0⁺) is
   computed using u(0).

In conclusion, neither of these choices is right or wrong; each choice just
carries its own implications.

@ingroup systems
*/

}  // namespace systems
}  // namespace drake
