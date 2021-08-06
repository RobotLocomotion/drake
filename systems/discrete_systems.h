
namespace drake {
namespace systems {

// If you modify this example, be sure to update the matching unit test in
// systems/analysis/test/simulator_test.cc.
// TODO(sherm1) When PR #10132 lands, beautify this example and use SignalLogger
// for output rather than std::cout.

/** @addtogroup discrete_systems
@brief This page describes discrete systems modeled by difference equations
(contrast to continuous systems modeled by ordinary differential equations)
as well as considerations for implementing these systems in Drake.

The state space dynamics of a discrete system is:
```
    x_{n+1} = f(n, x_n, u_n)    // update
    y_n     = g(n, x_n, u_n)    // output
    x_0     = x_init            // initialize
```
(We're using LaTeX underscore notation for subscripts, where x_0 means x₀.)

Here n ∈ ℕ is the step number (typically starting at zero), x is the
discrete-time state variable ("discrete time" refers to the countability of the
elements of the sequence, x_0, x_1, ..., x_n and not the values that x can
take), y is the desired
output, and u is an external input. f(.) and g(.) are the _update_ and _output_
functions, respectively. Any of these quantities can be vector-valued. The
subscript notation (e.g., x_0) is used to show that the state, input, and output
result from a discrete process. We use square bracket notation, e.g. x[1] to
designate particular elements of a vector-valued quantity (indexing from 0).
Combined, x_1[3] would be the value of the fourth element of the x vector,
evaluated at step n=1.

<h3>A pedagogical example: simple difference equation</h3>

The following class implements in Drake the simple discrete system
```
    x_{n+1} = x_n + 1
    y_n     = 10 x_n
    x_0     = 0
```
which should generate the sequence `S = 0 10 20 30 ...` (that is, `S_n = 10*n`).

@code{.cpp}
class ExampleDiscreteSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExampleDiscreteSystem)

  ExampleDiscreteSystem() {
    DeclareDiscreteState(1);  // Just one state variable, x[0], default=0.

    // Update to x_{n+1}, using a Drake "discrete update" event (occurs
    // at the beginning of step n+1).
    DeclarePeriodicDiscreteUpdateEvent(kPeriod, kOffset,
                                       &ExampleDiscreteSystem::Update);

    // Present y_n (=S_n) at the output port.
    DeclareVectorOutputPort("Sn", 1, &ExampleDiscreteSystem::Output);
  }

  static constexpr double kPeriod = 1 / 50.;  // Update at 50Hz (h=1/50).
  static constexpr double kOffset = 0.;       // Trigger events at n=0.

 private:
  void Update(const systems::Context<double>& context,
              systems::DiscreteValues<double>* xd) const {
    const double x_n = context.get_discrete_state()[0];
    (*xd)[0] = x_n + 1.;
  }

  void Output(const systems::Context<double>& context,
              systems::BasicVector<double>* result) const {
    const double x_n = context.get_discrete_state()[0];
    const double S_n = 10 * x_n;
    (*result)[0] = S_n;
  }
};
@endcode

The following code fragment can be used to step this system forward:
@code
  // Build a Diagram containing the Example system and a data logger that
  // samples the Sn output port exactly at the update times.
  DiagramBuilder<double> builder;
  auto example = builder.AddSystem<ExampleDiscreteSystem>();
  auto logger = LogOutput(example->GetOutputPort("Sn"), &builder);
  logger->set_publish_period(ExampleDiscreteSystem::kPeriod);
  auto diagram = builder.Build();

  // Create a Simulator and use it to advance time until t=3*h.
  Simulator<double> simulator(*diagram);
  simulator.AdvanceTo(3 * ExampleDiscreteSystem::kPeriod);

  // Print out the contents of the log.
  for (int n = 0; n < logger->sample_times().size(); ++n) {
    const double t = logger->sample_times()[n];
    std::cout << n << ": " << logger->data()(0, n)
        << " (" << t << ")\n";
  }
@endcode
The above yields the following output:
```
    0: 0 (0)
    1: 10 (0.02)
    2: 20 (0.04)
    3: 30 (0.06)
```

<h3>Modeling discrete systems in Drake's hybrid systems framework</h3>

Purely-discrete systems produce values only intermittently. For example, the
system above generates values only at integer values of n: <pre>

     y_n
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
conversion to time used by the example above as a second horizontal axis,
assuming h=0.02 seconds.

However, since Drake simulations advance through _continuous_ time, it must be
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
       (Drake uses this method.)              (Not used in Drake.)

          Figure 2: two ways to make a continuous function from
          a discrete one, using sample-and-hold.
</pre>
In the figure, the ○ markers show the function value at time t _before_ the
update function is invoked, while the ● markers show the value _after_ the
update. In (a), the ○ markers coincide with the original discrete values, while
in (b), the ● markers do.

You might expect that 2(b) would be the most natural mapping from the
discrete system to a continuous one. In practice, however, it is problematic
for mixed discrete/continuous (hybrid) systems so Drake uses the mapping in
2(a). The advantage of 2(a) is that the hybrid update function
`x_{n+1} = f(t,n,x_n,u(t))` is invoked at time `t=n*h`, while in 2(b) it would be
invoked at time `t=(n+1)*h`. That would make it difficult to coordinate discrete
and continuous signals.

Drake's choice of 2(a) dictates what value a discrete quantity will have when
evaluated at times _between_ update times. In particular, consider a discrete
variable x evaluated during a simulation from a publish, update, or derivative
function at times `t ∈ (n*h, (n+1)*h]`. x will be seen to have value
`x(t) = x_{n+1}` (_not_ `x_n`). You can see that clearly by inspection of
Figure 2(a).

<h3>Timing of publish vs. discrete update events in Drake</h3>

A discrete system viewed in continuous time does not have a unique value at its
sample times. In Figure 2, each pair of ○ and ● symbols shows two possible
values at the same time. For a given sample time t, we use the notation x⁻(t) to
denote the "pre-update" value of the state x, and x⁺(t) to denote the
"post-update" value of x. So x⁻(t) is the value of x at time t _before_ discrete
variables are updated, and x⁺(t) the value of x at time t _after_ they are
updated. Thus if we have `t = n*h` as in the discussion above, then
`x⁻(t) = x_n` and `x⁺(t) = x_{n+1}`. State-dependent computations are affected
by the scheduling of these updates. For example, evaluating an input u(t) yields
u⁻(t) before discrete updates, and u⁺(t) afterwards, meaning that the input
evaluation is carried out using x⁻(t) or x⁺(t), respectively.

With those distinctions drawn, we can define Drake's state update behavior
during a time step:
 - `Publish` events at time t see x⁻(t), so if a publish event handler evaluates
   an input it sees u⁻(t). This occurs at the end of a step, shown as ○ markers
   in Figure 2.
 - `Update` events (of all kinds) at time t also see x⁻(t) and u⁻(t), and
   produce x⁺(t). This occurs at the start of the next step, shown as ● markers
   in Figure 2.
 - `Continuous` update (numerical integration and time advancement) starts with
   x⁺(t). Input and derivative evaluations will occur repeatedly as the time
   and continuous state advance. Each evaluation will be performed using
   updated values for continuous states xc and time. However, the discrete
   variables (state partitions xd and xa) are held constant at their x⁺(t)
   values, that is, at xd⁺(t) and xa⁺(t).

If you define periodic events starting at t=0 as we did in the
example above, the first publish event occurs at the end of initialization,
while the first discrete update event occurs at the beginning of the first step,
followed by continuous time and state advancement.

@see drake::systems::Simulator for more details.

@ingroup systems
*/

}  // namespace systems
}  // namespace drake
