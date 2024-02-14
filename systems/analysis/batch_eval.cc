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
    const System<T>& system,
    const std::vector<std::unique_ptr<Context<T>>>& context_pool,
    const Eigen::Ref<const RowVectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& states,
    const Eigen::Ref<const MatrixX<T>>& inputs, int num_time_steps,
    std::variant<systems::InputPortSelection, InputPortIndex> input_port_index,
    Parallelism parallelize) {
  DRAKE_DEMAND(system.IsDifferenceEquationSystem());
  DRAKE_DEMAND(!context_pool.empty());
  for (int i = 0; i < ssize(context_pool); ++i) {
    system.ValidateContext(*context_pool[i]);
  }
  const int num_evals = times.cols();
  DRAKE_DEMAND(states.rows() ==
               context_pool[0]->get_discrete_state_vector().size());
  DRAKE_DEMAND(states.cols() == num_evals);
  const InputPort<T>* input_port =
      system.get_input_port_selection(input_port_index);
  DRAKE_DEMAND(input_port->get_data_type() == PortDataType::kVectorValued);
  DRAKE_DEMAND(inputs.rows() == input_port->size());
  DRAKE_DEMAND(inputs.cols() == num_evals);
  DRAKE_DEMAND(num_time_steps > 0);

  const int num_threads_to_use =
      std::min<int>(ssize(context_pool), parallelize.num_threads());

  MatrixX<T> next_states = MatrixX<T>::Zero(states.rows(), num_evals);

  const auto calc_next_state = [&](const int thread_num, const int64_t i) {
    context_pool[thread_num]->SetTime(times(i));
    next_states.col(i) = states.col(i);
    input_port->FixValue(context_pool[thread_num].get(), inputs.col(i));

    for (int step = 0; step < num_time_steps; ++step) {
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
    const System<T>& system,
    const std::vector<std::unique_ptr<Context<T>>>& context_pool,
    const Eigen::Ref<const RowVectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& states,
    const Eigen::Ref<const MatrixX<T>>& inputs,
    std::variant<systems::InputPortSelection, InputPortIndex> input_port_index,
    Parallelism parallelize) {
  DRAKE_DEMAND(!context_pool.empty());
  const int num_evals = times.cols();
  DRAKE_DEMAND(!context_pool.empty());
  for (int i = 0; i < ssize(context_pool); ++i) {
    system.ValidateContext(*context_pool[i]);
  }
  DRAKE_DEMAND(states.rows() == context_pool[0]->num_continuous_states());
  DRAKE_DEMAND(states.cols() == num_evals);
  const InputPort<T>* input_port =
      system.get_input_port_selection(input_port_index);
  DRAKE_DEMAND(input_port->get_data_type() == PortDataType::kVectorValued);
  DRAKE_DEMAND(inputs.rows() == input_port->size());
  DRAKE_DEMAND(inputs.cols() == num_evals);

  const int num_threads_to_use =
      std::min<int>(ssize(context_pool), parallelize.num_threads());

  MatrixX<T> derivatives = MatrixX<T>::Zero(states.rows(), num_evals);

  const auto calc_derivatives = [&](const int thread_num, const int64_t i) {
    context_pool[thread_num]->SetTime(times(i));
    context_pool[thread_num]->SetContinuousState(states.col(i));
    input_port->FixValue(context_pool[thread_num].get(), inputs.col(i));

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
