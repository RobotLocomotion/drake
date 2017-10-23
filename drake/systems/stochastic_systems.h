
namespace drake {
namespace systems {

/**
@page stochastic_systems Stochastic Systems

This page describes the implementation details of modeling a
stochastic system in Drake and writing algorithms that explicitly
leverage the stochastic modeling framework.

A system in Drake can be viewed as having the state-space dynamics

  x[n+1] = f(p; n, x[n], u[n], w[n]), 

  y[n] = g(p; n, x[n], u[n], w[n]), 

where n is the time index, x is the state, y is the output, u is the
input, and p are the (constant) parameters.  This form also calls out
w explicitly as a random "disturbance" input.  These random inputs are
implemented and evaluated using exactly the same methods as the
deterministic inputs; any input port can optionally be annotated as
"random" when they are being declared (see
System<T>::DeclareInputPort() ).  Once randomness exists in a system,
many signals will throughout the diagram become random variables, but
this label is used to denote the "point of entry" for an independent
random variable.

Note: For simplicity, I've written only a simple discrete-time system
form above, but the same model holds for continuous-time systems, and
multi-rate systems, and systems with multiple input/output ports as
well, precisely because w is treated exactly as an additional input
throughout the system classes.  Continuous-time random signals must
be treated with some care; see internal::RandomSource for details on
how they are treated for the purpopses of simulation.

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
InputPortDescriptor to find the input ports that are labeled as
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
(which calls both of these).  The implementations of
System<T>::SetRandomState() and System<T>::SetRandomParameters() are
expected to call the random number generators in the C++ Standard
Library.

*/

}  // namespace systems
}  // namespace drake
