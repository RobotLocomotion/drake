#include "drake/systems/analysis/batch_eval.h"

#include <algorithm>
#include <memory>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;

template <typename T>
MatrixX<T> BatchEvalUniquePeriodicDiscreteUpdate(
    const System<T>& system, const Context<T>& context,
    const Eigen::Ref<const RowVectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& states,
    const Eigen::Ref<const MatrixX<T>>& inputs, int num_time_steps,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    Parallelism parallelize) {
  system.ValidateContext(context);
  double time_step{0.0};
  DRAKE_THROW_UNLESS(system.IsDifferenceEquationSystem(&time_step));
  const int num_evals = times.size();
  DRAKE_THROW_UNLESS(states.rows() ==
                     context.get_discrete_state_vector().size());
  DRAKE_THROW_UNLESS(states.cols() == num_evals);
  const InputPort<T>* input_port =
      system.get_input_port_selection(input_port_index);
  if (input_port) {
    DRAKE_THROW_UNLESS(input_port->get_data_type() ==
                       PortDataType::kVectorValued);
    DRAKE_THROW_UNLESS(inputs.rows() == input_port->size());
    DRAKE_THROW_UNLESS(inputs.cols() == num_evals);
  }
  DRAKE_THROW_UNLESS(num_time_steps > 0);

  const int num_threads_to_use = parallelize.num_threads();
  std::vector<std::unique_ptr<Context<T>>> context_pool(num_threads_to_use);

  MatrixX<T> next_states = MatrixX<T>::Zero(states.rows(), num_evals);

  const auto calc_next_state = [&](const int thread_num, const int64_t i) {
    if (!context_pool[thread_num]) {
      context_pool[thread_num] = context.Clone();
    }
    next_states.col(i) = states.col(i);

    // The input port stays fixed for all of the steps.
    if (input_port) {
      input_port->FixValue(context_pool[thread_num].get(), inputs.col(i));
    }
    for (int step = 0; step < num_time_steps; ++step) {
      // Set the time and state for this step.
      context_pool[thread_num]->SetTime(times(i) + step * time_step);
      context_pool[thread_num]->SetDiscreteState(next_states.col(i));
      next_states.col(i) =
          system.EvalUniquePeriodicDiscreteUpdate(*context_pool[thread_num])
              .value();
    }
  };

  StaticParallelForIndexLoop(DegreeOfParallelism(num_threads_to_use), 0,
                             num_evals, calc_next_state,
                             ParallelForBackend::BEST_AVAILABLE);

  return next_states;
}

template <typename T>
MatrixX<T> BatchEvalTimeDerivatives(
    const System<T>& system, const Context<T>& context,
    const Eigen::Ref<const RowVectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& states,
    const Eigen::Ref<const MatrixX<T>>& inputs,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    Parallelism parallelize) {
  system.ValidateContext(context);
  const int num_evals = times.size();
  DRAKE_THROW_UNLESS(states.rows() == system.num_continuous_states());
  DRAKE_THROW_UNLESS(states.cols() == num_evals);
  const InputPort<T>* input_port =
      system.get_input_port_selection(input_port_index);
  if (input_port) {
    DRAKE_THROW_UNLESS(input_port->get_data_type() ==
                       PortDataType::kVectorValued);
    DRAKE_THROW_UNLESS(inputs.rows() == input_port->size());
    DRAKE_THROW_UNLESS(inputs.cols() == num_evals);
  }

  const int num_threads_to_use = parallelize.num_threads();
  std::vector<std::unique_ptr<Context<T>>> context_pool(num_threads_to_use);

  MatrixX<T> derivatives = MatrixX<T>::Zero(states.rows(), num_evals);

  const auto calc_derivatives = [&](const int thread_num, const int64_t i) {
    if (!context_pool[thread_num]) {
      context_pool[thread_num] = context.Clone();
    }
    context_pool[thread_num]->SetTime(times(i));
    context_pool[thread_num]->SetContinuousState(states.col(i));
    if (input_port) {
      input_port->FixValue(context_pool[thread_num].get(), inputs.col(i));
    }
    derivatives.col(i) =
        system.EvalTimeDerivatives(*context_pool[thread_num]).CopyToVector();
  };

  StaticParallelForIndexLoop(DegreeOfParallelism(num_threads_to_use), 0,
                             num_evals, calc_derivatives,
                             ParallelForBackend::BEST_AVAILABLE);

  return derivatives;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&BatchEvalUniquePeriodicDiscreteUpdate<T>, &BatchEvalTimeDerivatives<T>));

}  // namespace systems
}  // namespace drake
