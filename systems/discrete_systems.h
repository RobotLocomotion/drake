
namespace drake {
namespace systems {

/** @addtogroup discrete_systems
@brief This page describes discrete systems modeled by difference equations
(contrast to continuous systems modeled by ordinary differential equations)
as well as considerations for implementing these systems in Drake.

The state space dynamics of a discrete system is:
@verbatim
x[n+1] = f(p; n, x[n], u[n])
y[n] = g(p; n, x[n], u[n])
@endverbatim

where `n` ∈ ℤ is the variable iterate (typically starting at zero), `x` ∈ ℝ is
the discrete state variable ("discrete" refers to the countability of the
elements of the sequence, x[1], ..., x[n] and not the values that x can take),
`y` is the output, `u` is the input, and `p` are the (constant)
parameters. `f(.)` and `g(.)` are the _process_ and _output_ models,
respectively. The bracket notation (e.g., `x[0]`) are used to show that
the state and output are a function of a discrete process. 

TODO: Finish me.
@image html framework/images/discrete.svg "Figure 1: Plots of state and output of a discrete system. This system shows the evolution of ..." 


A simple difference equation is:
@verbatim
x[n+1] = x[n] + 1
@endverbatim

<h2>Drake models hybrid systems</h2>
In Drake, discrete systems possess one or more discrete variables
(DiscreteValues) and are usually updated at a specified periodicity (which
we will denote `h`) as time advances. For example, the following class
describes a simple discrete system whose evolution is described by the above
difference equation:
@code
class SimpleDiscreteSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleDiscreteSystem)
  SimpleDiscreteSystem() {
    this->DeclarePeriodicDiscreteUpdate(kPeriod, kOffset);
    this->DeclareDiscreteState(1 /* single state variable */);
  }

 private:
  void DoCalcDiscreteVariableUpdates(
      const Context<double>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>&,
      DiscreteValues<double>* x_next) const override {
    const double x = context.get_discrete_state()[0];
    (*x_next)[0] = x + 1;
  }
  const double kPeriod = 1.0;  // Update every second (h=1).
  const double kOffset = 1.0;  // First update is at one second.
};
@endcode

 Using time in place of iteration count allows Drake to
 model the evolution of hybrid continuous/discrete systems using a single
 independent variable (time). But this decision gives rise to particular
 implications which we will now describe.

 <h3>(a) Discrete variables take on two values at their update times</h3>
 The discrete system represented in our pedagogical example takes on two
 values at `t = 1.0`: `x[0]` (pre-update `x`) and `x[1]` (post-update `x`).
 We distinguish between pre-update and post-update times using `t⁻` and `t⁺`,
 respectively. Likewise, we can distinguish between the value of `x` at pre-
 and post-update times as `x(1.0⁻) = x[0]` and `x(1.0⁺) = x[1]`. The
 parenthesis notation refers to the value of the variable at a _time_ while
 the bracket notation refers to the value of the variable at an _iteration_.

 <h3>(b) Should the first update be at time zero?</h3>
 The discrete system `x[n+1] = x[n] + 1` can be modeled two different ways
 (resulting in slightly different evolutions) in Drake: the first update can
 happen at time zero or it can happen at time `h`. The two sequences below
 show the result of applying both strategies to `x[0] = x(0⁻) = 0`.
 @verbatim
 x(0⁺) = 1, x(1⁺) = 2, ..., x(i⁺) = i + 1    (first update at time zero)
 x(1⁺) = 1, x(2⁺) = 2, ..., x(i⁺) = i        (first update at time h)
 @endverbatim
 Note that the first element in each sequence is `x[1]`, the second `x[2]`,
 etc. Although the aggregate behavior differs, the two sequences do match
 at the pre-update time for the first sequence and the post-update time of
 the second sequence, i.e., `x(1⁻)` {first} `= x(1⁺)` {second}` = 1`.

 In order to understand the full implications of the choice of update
 time, we must examine how discrete and continuous systems interact.
 Consider the hybrid continuous-discrete system: `x[n+1] = x[n] + u(t)` where
 `h = 1`, `x[0] = 0`, and `u(t) = t`. Applying the two strategies to
 initial conditions `x[0] = x(0⁻) = 0` (i.e., advancing the system through
 time using Drake's Simulator)  yields:
 @verbatim
 x(0⁺) = 0, x(1⁺) = 1, x(2⁺) = 3 ...        (first update at time zero)
 x(1⁺) = 1, x(2⁺) = 3, x(3⁺) = 6 ...        (first update at time h)
 @endverbatim
 See the class documentation for Simulator for a detailed understanding of
 the stepping process.

 Examining the two pedagogical examples, the following characteristics
 should become apparent.

 _For the case of updates starting at time zero_:
 1. Advancing time and state to that at `t = 1e-16` results in the discrete
    value being advanced to `x[1]`.
 2. The input is evaluated at the left end of each discrete interval, e.g.,
    x(0⁺) is computed using u(0).

 _For the case of updates starting at time `h`_:
 1. The discrete value is only advanced to `x[1]` when `t=h`.
 2. The input is evaluated at the right end of each interval, e.g.,
    x(h⁺) is computed using u(h).

@ingroup systems
*/

}  // namespace systems
}  // namespace drake
