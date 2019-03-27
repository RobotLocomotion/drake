#pragma once

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace analysis {

/***
 * Defines a factory method that constructs a Simulator (with an owned System)
 * using the supplied RandomGenerator as the only source of randomness.
 *
 * Note that in many interesting cases, the SimulatorFactory may simply ignore
 * the RandomGenerator argument and return the Simulator object
 * deterministically, because randomness may also be introduced *inside* the
 * simulation (by SetRandomContext and/or random input ports).
 *
 * Also consider that the System referenced by the Simulator returned by this
 * method must have a lifetime that lasts beyond any calls this Simulator.
 * Having the Simulator own the System (by calling the unique_ptr version of
 * the constructor) is one convenient solution.
 */
typedef std::function<std::unique_ptr<Simulator<double>>(
    RandomGenerator* generator)>
    SimulatorFactory;

/***
 * Defines an arbitrary scalar function of the Context.  This is used in the
 * RandomSimulation and MonteCarloSimulation tools below as a way of
 * defining the output random variable of interest -- the
 * ScalarSystemFunction is evaluated with the final conditions of the
 * simulation and the double that is returned is the value of the random
 * variable.  (Although there is no randomness in the ScalarSystemFunction
 * itself, in the RandomSimulation case the final Context will be random, so
 * functions of that context will also be random).
 */
typedef std::function<double(const System<double>& system,
                             const Context<double>& context)>
    ScalarSystemFunction;

/**
 * Run a deterministic simulation of a (stochastic) System using the @p
 * generator to instantiate all "random" quantities.
 *
 * In pseudo-code, this algorithm implements:
 * @code
 *   simulator = make_simulator(generator)
 *   simulator.get_system().SetRandomContext(generator)
 *   simulator.AdvanceTo(final_time)
 *   return output(simulator.get_context())
 * @endcode
 *
 * @param make_simulator  Callers to this method define a stochastic simulation
 * by providing the @p make_simulator factory method to return a Simulator
 * using the supplied RandomGenerator as the only source of randomness.
 * This interface was designed to support cases where the System/Diagram is
 * random (not only the Context), e.g. in the case where are variable number
 * of objects are added to a multibody simulation.
 *
 * @param output  The scalar random variable output, denoted @p output, is
 * defined as a function of the Simulator's System's Context, evaluated at
 * the @p final_time.  Monte-Carlo investigations that studying the details
 * of an entire trajectory can still use this interface, e.g. by including a
 * "runtime monitor" System that latches the worst-case deviation of a
 * specification into it's Context to be queried at the final time.
 *
 * @param final_time  The time that each instance of the Simulator is
 * stepped to.  In many cases, this will be equivalent to the duration of
 * the simulation, but it need not be because SetRandomContext() could
 * initialize the time to a non-zero value, or an event could trigger
 * premature termination of the simulation (see #4447).
 *
 * @param generator Random number generator to be used to generate the
 * random samples.
 *
 * @return the @p output evaluated from the Context at @p final_time.
 *
 * @ingroup analysis
 */
double RandomSimulation(const SimulatorFactory& make_simulator,
                        const ScalarSystemFunction& output, double final_time,
                        RandomGenerator* generator);

/**
 * A snapshot of the generator used to produce the random simulation.  Use,
 * e.g.,
 * @code
 *   RandomGenerator generator(result.generator_snapshot)
 *   RandomSimulation(make_simulator, output, final_time, &generator)
 * @endcode
 * for a deterministic playback of the sampled simulation. Use the operator<<()
 * operator>>() methods provided for the generator, which implements the
 * Standard Template Library <a href=
 *   "https://en.cppreference.com/w/cpp/named_req/RandomNumberEngine">
 * RandomNumberEngine concept</a>, if you wish to serialize the results.
 * Note that the generator "snapshots" are returned as const, because any
 * non-const operations on that object may advance the state of the
 * generator (making it no-longer valuable in reproducing the simulation).
 */
struct RandomSimulationResult {
  explicit RandomSimulationResult(const RandomGenerator& generator,
                                  double value = 0.0)
      : generator_snapshot(generator), output(value) {}

  const RandomGenerator generator_snapshot;
  double output{};
};

/**
 * Generate samples of a scalar random variable output by running many
 * random simulations drawn from independent samples of the
 * distributions governing the stochastic simulation.
 *
 * In pseudo-code, this algorithm implements:
 * @code
 *   for i=1:num_samples
 *     const generator_snapshot = deepcopy(generator)
 *     output = RandomSimulation(..., generator)
 *     data(i) = std::pair(generator_snapshot, output)
 *   return data
 * @endcode
 *
 * @see RandomSimulation() for details about @p make_simulator, @p output,
 * and @p final_time.
 *
 * @param num_samples Number of independent samples to draw from the
 * distribution (and equivalently, the number of simulations to run).
 *
 * @param generator Random number generator to be used to generate the
 * random samples.  If null, then a new RandomGenerator will be allocated
 * and used internally (and repeated calls to this method will return
 * identical results).  To produce statistically "independent" samples on a
 * future call to MonteCarloSimulation, you should make repeated uses of the
 * same RandomGenerator object.
 *
 * @returns a list of RandomSimulationResult's.
 *
 * @ingroup analysis
 */
// TODO(russt): Consider generalizing this with options (e.g. setting the
// number of simulators, number of samples per simulator, number of parallel
// threads, ...).
std::vector<RandomSimulationResult> MonteCarloSimulation(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    double final_time, int num_samples, RandomGenerator* generator = nullptr);

}  // namespace analysis
}  // namespace systems
}  // namespace drake
