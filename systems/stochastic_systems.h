
namespace drake {
namespace systems {

/** @addtogroup stochastic_systems
@brief This page describes the implementation details of modeling a
stochastic system in Drake and writing algorithms that explicitly
leverage the stochastic modeling framework.

A discrete-time system in Drake can be viewed as having the state-space 
dynamics

  x_{n+1} = f(p; n, x_n, u_n, w_n),

  y_n = g(p; n, x_n, u_n, w_n),

where n is the step index, x is the state, y is the output, u is the
input, and p are the (constant) parameters.  This form also calls out
w explicitly as a random "disturbance" input.  These random inputs are
implemented and evaluated using exactly the same methods as the
deterministic inputs; any input port can optionally be annotated as
"random" when they are being declared (see
System<T>::DeclareInputPort() ).  Once randomness exists in a system,
many signals will throughout the diagram become random variables, but
this label is used to denote the "point of entry" for an independent
random variable.

@note For simplicity, I've written only a simple discrete-time system
form above, but the same model holds for continuous-time systems, and
multi-rate systems, and systems with multiple input/output ports as
well, precisely because w is treated exactly as an additional input
throughout the system classes.  Continuous-time random signals must
be treated with some care; see internal::RandomSource for details on
how they are treated for the purposes of simulation.

The rule in Drake is that every method that can be called during the
lifetime of a simulation, (e.g. calculating discrete updates, time
derivatives, and/or outputs) *must* be a completely deterministic
function.  Any randomness must come in through a random input port.
The only exception to this rule is the one specially-implemented
internal::RandomSource system, which goes to some length to store the
state of the random number generator in its Context so that all
simulation and analysis methods are deterministic given a Context.  In
almost every application, random input ports will be wired up to
internal::RandomSource system blocks; we have provided the
AddRandomInputs() method to facilitate this.

Algorithms written for Systems can query the property of the
InputPort to find the input ports that are labeled as
random, and the random vector distribution type.  The list of
supported distributions for random input ports is intentionally very
limited (to simplify algorithm development); we place the burden on
the System author to e.g. transform a Gaussian random input with zero
mean and unit covariance into the desired shape inside the update and
output methods.

In order to specify distributions over random initial conditions and
random parameters, System classes may override the methods
System<T>::SetRandomState() and System<T>::SetRandomParameters().
Algorithms written for systems may call System<T>::SetRandomContext()
(which calls both of these).  These methods must be deterministic
functions of their input arguments -- a (mutable) drake::RandomGenerator is
passed in and must be the only source of "randomness". Their implementations
are expected to draw samples from random distributions satisfying the C++
Standard Library <a href=
"https://en.cppreference.com/w/cpp/named_req/RandomNumberDistribution">
RandomNumberDistribution concept</a>.  Because these distributions can have
internal state, we encourage authors to allocate the distribution locally
inside the implementation of the SetRandomState() or SetRandomParameters()
methods; do not be tempted to create a mutable member variable of your System.

Finally, in some cases the actual constitution of a System is random.
A common example of this is when we perform a multibody simulation with
a different number of objects generated in the environment for each random
simulation.  APIs to support this functionality use a
systems::analysis::SimulatorFactory method which constructs the random
System and its simulation parameters as a deterministic function
of the drake::RandomGenerator.

@see systems::analysis::MonteCarloSimulation
@ingroup systems
*/

}  // namespace systems
}  // namespace drake
