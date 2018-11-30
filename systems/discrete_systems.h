
namespace drake {
namespace systems {

/** @addtogroup discrete_systems Discrete Systems
@brief This page describes discrete systems modeled by difference equations
(contrast to continuous systems modeled by ordinary differential equations)
as well as considerations for implementing these systems in Drake.

The state space dynamics of a discrete system is:
```
    xₙ₊₁ = f(n, xₙ, uₙ)    // update
    yₙ   = g(n, xₙ, uₙ)    // output
    x₀   = xᵢₙᵢₜ           // initialize
```

where n ∈ ℕ is the step number (typically starting at zero), x is the discrete
state variable ("discrete" refers to the countability of the elements of the
sequence, x₀, x₁, ..., xₙ and not the values that x can take), y is the desired
output, and u is an external input. f(.) and g(.) are the _update_ and _output_
functions, respectively. Any of these quantities can be vector-valued. The
subscript notation (e.g., x₀) is used to show that the state, input, and output
result from a discrete process. We use square bracket notation, e.g. x[1] to
designate particular elements of a vector-valued quantity (indexing from 0).
Combined, x₁[3] would be the value of the fourth element of the x vector,
evaluated at step n=1. In code we use a Latex-like underscore to indicate the
step number, so we write x_1[3] to represent x₁[3].

<h3>A pedagogical example: simple difference equation</h3>

The following class implements in Drake the simple discrete system
```
    xₙ₊₁ = xₙ + 1
    yₙ   = 10 xₙ
    x₀   = 0
```
which should generate the sequence `S = 0 10 20 30 ...` (that is, `Sₙ = 10*n`).

@code{.cpp}
class SimpleDiscreteSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleDiscreteSystem)

  SimpleDiscreteSystem() {
    DeclareDiscreteState(1);  // Just one state variable, x[0].

    // Output yₙ using a Drake "publish" event (occurs at the end of step n).
    DeclarePeriodicEvent(kPeriod, kOffset,
                         systems::PublishEvent<double>(
                             [this](const systems::Context<double>& context,
                                    const systems::PublishEvent<double>&) {
                               Output(context);
                             }));

    // Update to xₙ₊₁ (x_np1), using a Drake "discrete update" event (occurs
    // at the beginning of step n+1).
    DeclarePeriodicEvent(kPeriod, kOffset,
                         systems::DiscreteUpdateEvent<double>(
                             [this](const systems::Context<double>& context,
                                    const systems::DiscreteUpdateEvent<double>&,
                                    systems::DiscreteValues<double>* x_np1) {
                               x_np1->get_mutable_vector()[0] =
                                   Update(context);
                             }));
  }

  // Set initial condition x₀ = 0.
  void Initialize(systems::Context<double>* context) const {
    context->get_mutable_discrete_state()[0] = 0.;
  }

  static constexpr double kPeriod = 0.02;  // Update at 50Hz (h=1/50).
  static constexpr double kOffset = 0.;    // Trigger events starting at n=0.

 private:
  // Update function xₙ₊₁ = f(n, xₙ).
  double Update(const systems::Context<double>& context) const {
    const double x_n = GetX(context);
    return x_n + 1.;
  }

  // Output function yₙ = g(n, xₙ). (Here, just writes 'n: Sₙ (t)' to cout.)
  void Output(const systems::Context<double>& context) const {
    const double t = context.get_time();
    const int n = static_cast<int>(std::round(t / kPeriod));
    const double S_n = 10 * GetX(context);  // 10 xₙ[0]
    std::cout << n << ": " << S_n << " (" << t << ")\n";
  }

  double GetX(const Context<double>& context) const {
    return context.get_discrete_state()[0];
  }
};
@endcode

Stepping this system forward using the following code fragment:
@code
SimpleDiscreteSystem system;
Simulator<double> simulator(system);
system.Initialize(&simulator.get_mutable_context());
simulator.StepTo(3 * SimpleDiscreteSystem::kPeriod);
@endcode
yields the following output:
```
    0: 0 (0)
    1: 10 (0.02)
    2: 20 (0.04)
    3: 30 (0.06)
```

<h3>Modeling discrete systems in Drake's hybrid systems framework</h3>

Purely-discrete systems produce values only intermittently. For example, the
system above generates values only at integer values of n: <pre>

     yₙ
      |
   30 |              ●
      |              ┆             Figure 1. The discrete-valued system
   20 |         ●    ┆             from the example above.
      |         ┆    ┆
   10 |    ●    ┆    ┆
      |    ┆    ┆    ┆
    0 ●----+----+----+--
      0    1    2    3   n
      +----+----+----+--
      0   .02  .04  .06  t
</pre>

Drake's simulator is for hybrid systems, that is, systems that advance through
_time_, and evolve in time both continuously (flow) and discretely (jump). It
is easy enough to use time to represent the discrete steps n, by the conversion
`t=n*h` where h is a periodic sampling time. In Figure 1 we've shown the
conversion to time used by the example above as a second horizontal axis.
However, since Drake simulations advance through continuous time, it must be
possible to obtain the values of all state variables and outputs at _any_ time
t, not just at discrete times. So the question arises: what is the value of y(t)
for values of t in between the sample times shown above? The answer doesn't
matter for the example above, but becomes significant when we mix continuous and
discrete systems, since they are typically interdependent.

Sample-and-hold is the most common way to go from a discrete value to a
continuous one. There are two equally-plausible ways to use sample-and-hold to
fill in the gaps between the discrete sample times above, shown in
Figure 2: <pre>

      y(t)             ●━                    y(t)
        |              ┆                       |
     30 |         ●━━━━○                    30 |              ●━
        |         ┆                            |              ┆
     20 |    ●━━━━○                         20 |         ●━━━━○
        |    ┆                                 |         ┆
     10 ●━━━━○                              10 |    ●━━━━○
        ┆                                      |    ┆
      0 ○----+----+----+-- t                 0 ●━━━━○----+----+-- t
        0   .02  .04  .06                      0   .02  .04  .06

   (a) Sample at start of step.           (b) Sample at end of step.

          Figure 2: two ways to make a continuous function from
          a discrete one, using sample-and-hold.
</pre>
In the figure, the ○ markers show the function value at time t _before_ the
update function is invoked, while the ● markers show the value _after_ the
update. In (a), the ○ markers coincide with the original discrete values, while
in (b), the ● markers do.

Either of the above continuous functions can be produced easily with Drake's
periodic events, by choosing whether the first update occurs at t=0 or t=h.
However, for most-convenient intermixing of continuous and discrete elements,
we recommend the sampling shown in Figure 2(a), which is produced by allowing
the first events to occur at t=0 as we did in the example above. With that
method the update that advances the discrete system from step n to step n+1
occurs at time `t = n*h` as expected, allowing continuous quantities like u(t)
to be used in the update function. On the other hand, with the sampling in
Figure 2(b) that update occurs at time `t = (n+1)*h` instead, meaning that the
value `u(n*h)` would not be available unless it had been previously sampled.

<h3>Timing of Publish vs. DiscreteUpdate in Drake</h3>

A discrete system viewed in continuous time does not have a unique value at
its sample times. In Figure 2 the ○ and ● symbols show two possible values
at the same times. For a given sample time t, we use the notation t⁻ to denote
the "pre-update" time, and t⁺ to denote the "post-update" time, so y(t⁻) is the
value of y at time t _before_ discrete variables are updated, and y(t⁺) the
value of y at time t _after_ they are updated. Thus if we have `t = n*h` then
`y(t⁻) = yₙ` and `y(t⁺) = yₙ₊₁`.

You can think of tᵢ⁻ as the time at the end of the iᵗʰ time step, while tᵢ⁺ is
the time at the beginning of time step i+1. Initialization can then be
considered the 0ᵗʰ "time step", so t₀⁻ occurs at the end of initialization,
while t₀⁺ occurs at the start of the first time step.

With those distinctions drawn, we can define Drake's event behavior: `Publish`
events occur at t⁻ (end of previous step, ○ markers in Figure 2), while `Update`
events occur at the start of the current step to produce the t⁺ values
(● markers). So if you define periodic events starting at t=0 as we did in the
example above, the first Publish event occurs at the end of initialization,
while the first Update event occurs at the beginning of the first time step.
Both `DiscreteUpdate` and `UnrestrictedUpdate` events are handled at the
beginning of each step, with all unrestricted updates done first, followed by
all discrete updates. The Context supplied as input to those update methods will
contain the t⁻ (step n) values.

@see drake::systems::Simulator for more details.

@ingroup systems
*/

}  // namespace systems
}  // namespace drake
