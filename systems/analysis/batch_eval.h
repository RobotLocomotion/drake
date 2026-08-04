#pragma once

#include <variant>
#include <vector>

#include "drake/common/parallelism.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/** Evaluates the dynamics of a difference equation `system` at many times,
states, and inputs. See System<T>::EvalUniquePeriodicDiscreteUpdate().

Each column of `times`, `states`, and `inputs` will be associated with a single
evaluation of the dynamics. The return value will be a matrix with each column
corresponding to the next state of the system evaluated `num_time_steps *
time_step` seconds after the provided time, using the `time_step` that is
reported by System<T>::IsDifferenceEquationSystem().

@tparam T The scalar type of the system.
@param system The system to evaluate.
@param context A context associated with `system`, which can be used to pass
system parameters.
@param times A 1 x N vector of times at which to evaluate the dynamics.
@param states A num_states x N matrix of states at which to evaluate the
dynamics.
@param inputs A num_inputs x N matrix of inputs at which to evaluate the
dynamics, where num_inputs must match the size of the input port selected. If
input_port_index is set to InputPortSelection::kNoInput, then the inputs
argument will be ignored.
@param num_time_steps The returned value will be the state at `time +
time_step*num_time_steps`.
@param input_port_index The input port index to use for evaluating the
dynamics. The default is to use the first input if there is one. A specific
port index or kNoInput can be specified instead. The input port must be
vector-valued and have the same size as the number of rows in `inputs`.
@param parallelize The parallelism to use for evaluating the dynamics.

@return A matrix with each column corresponding to the next state at `time +
num_time_steps * time_step`.

@throws std::exception if `system.IsDifferenceEquationSystem()` is not true.
@throws std::exception if matrix shapes are inconsistent, with inputs required
only if an input port is provided.
*/
template <typename T>
MatrixX<T> BatchEvalUniquePeriodicDiscreteUpdate(
    const System<T>& system, const Context<T>& context,
    const Eigen::Ref<const RowVectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& states,
    const Eigen::Ref<const MatrixX<T>>& inputs, int num_time_steps = 1,
    std::variant<InputPortSelection, InputPortIndex> input_port_index =
        InputPortSelection::kUseFirstInputIfItExists,
    Parallelism parallelize = Parallelism::Max());

/** Evaluates the time derivatives of a `system` at many times, states, and
inputs.

Each column of `times`, `states`, and `inputs` will be associated with a single
evaluation of the dynamics. The return value will a matrix with the
corresponding time derivatives in each column. Any discrete variables in
`context` will be held constant across all evaluations.

@tparam T The scalar type of the system.
@param system The system to evaluate.
@param context A context associated with `system`, which can be used to pass
system parameters and discrete/abstract state.
@param times A 1 x N vector of times at which to evaluate the dynamics.
@param states A system.num_continuous_states() x N matrix of continuous states
at which to evaluate the dynamics.
@param inputs A num_inputs x N matrix of inputs at which to evaluate the
dynamics, where num_inputs must match the size of the input port selected. If
input_port_index is set to InputPortSelection::kNoInput, then the inputs
argument will be ignored.
@param input_port_index The input port index to use for evaluating the
dynamics. The default is to use the first input if there is one. A specific
port index or kNoInput can be specified instead. The input port must be
vector-valued and have the same size as the number of rows in `inputs`.
@param parallelize The parallelism to use for evaluating the dynamics.

@return A matrix with each column corresponding to the time derivatives.

@throws std::exception if matrix shapes are inconsistent, with inputs required
only if an input port is provided.
*/
template <typename T>
MatrixX<T> BatchEvalTimeDerivatives(
    const System<T>& system, const Context<T>& context,
    const Eigen::Ref<const RowVectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& states,
    const Eigen::Ref<const MatrixX<T>>& inputs,
    std::variant<InputPortSelection, InputPortIndex> input_port_index =
        InputPortSelection::kUseFirstInputIfItExists,
    Parallelism parallelize = Parallelism::Max());

}  // namespace systems
}  // namespace drake
