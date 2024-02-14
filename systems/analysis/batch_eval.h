#pragma once

#include <variant>
#include <vector>

#include "drake/common/parallelism.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/** Evaluates the dynamics of a difference equation `system` at many times,
states, and inputs. See System<T>::IsDifferenceEquationSystem().

For each column of `times`, `states`, and `inputs` will be associated with a
single evaluation of the dynamics. The return value will be the next state of
the system evaluated `time_step` seconds after the provided time.

@tparam T The scalar type of the system.
@param system The system to evaluate.
@param context_pool A vector of contexts associated with `system` to use for
evaluation. 
@param times A 1 x N vector of times at which to evaluate the dynamics.
@param states A num_states x N matrix of states at which to evaluate the
dynamics.
@param inputs A num_inputs x N matrix of inputs at which to evaluate the
dynamics.
@param num_time_steps The returned value will be the state at `time +
time_step*num_steps`.
@param input_port_index The input port index to use for evaluating the
dynamics. If a systems::InputPortSelection is provided, the first input port
will be used if it exists. If an InputPortIndex is provided, the input port
with that index will be used.
@param parallelize The parallelism to use for evaluating the dynamics. The
number of parallel threads will be `min(context_pool.size(),
parallelize.num_threads())`.

@return A matrix with each column corresponding to the next state at `time +
time_step`.

Note: If using parallel evaluation, then `system` must *not* have any elements
that are implemented in Python.

@pre `system.IsDifferenceEquationSystem()` returns true.
@pre `times.cols() == states.cols() == inputs.cols()`.
*/
template <typename T>
MatrixX<T> BatchEvalUniquePeriodicDiscreteUpdate(
    const System<T>& system,
    const std::vector<std::unique_ptr<Context<T>>>& context_pool,
    const Eigen::Ref<const RowVectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& states,
    const Eigen::Ref<const MatrixX<T>>& inputs, int num_time_steps = 1,
    std::variant<systems::InputPortSelection, InputPortIndex> input_port_index =
        systems::InputPortSelection::kUseFirstInputIfItExists,
    Parallelism parallelize = Parallelism::Max());

/** Evaluates the time derivatives of a `system` at many times,
states, and inputs.

For each column of `times`, `states`, and `inputs` will be associated with a
single evaluation of the dynamics. The return value will be the next state of
the system evaluated `time_step` seconds after the provided time.

@tparam T The scalar type of the system.
@param system The system to evaluate.
@param context The context to use for evaluation.
@param times A 1 x N vector of times at which to evaluate the dynamics.
@param states A num_states x N matrix of continuous states at which to evaluate
the dynamics.
@param inputs A num_inputs x N matrix of inputs at which to evaluate the
dynamics.
@param num_time_steps The returned value will be the state at `time +
time_step*num_steps`.
@param input_port_index The input port index to use for evaluating the
dynamics. If a systems::InputPortSelection is provided, the first input port
will be used if it exists. If an InputPortIndex is provided, the input port
with that index will be used.
@param parallelize The parallelism to use for evaluating the dynamics. The
number of parallel threads will be `min(context_pool.size(),
parallelize.num_threads())`.

@return A matrix with each column corresponding to the next state at `time +
time_step`.

Note: If using parallel evaluation, then `system` must *not* have any
elements that are implemented in Python.

@pre `!context_pool.empty()`.
@pre `times.cols() == states.cols() == inputs.cols()`.
*/
template <typename T>
MatrixX<T> BatchEvalTimeDerivatives(
    const System<T>& system,
    const std::vector<std::unique_ptr<Context<T>>>& context_pool,
    const Eigen::Ref<const RowVectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& states,
    const Eigen::Ref<const MatrixX<T>>& inputs,
    std::variant<systems::InputPortSelection, InputPortIndex> input_port_index =
        systems::InputPortSelection::kUseFirstInputIfItExists,
    Parallelism parallelize = Parallelism::Max());

}  // namespace systems
}  // namespace drake
