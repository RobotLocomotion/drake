#include "drake/planning/sampling_based/dev/path_processor.h"

#include <chrono>
#include <functional>

#include <common_robotics_utilities/path_processing.hpp>
#include <common_robotics_utilities/utility.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace planning {
using common_robotics_utilities::path_processing::ResamplePath;
using common_robotics_utilities::path_processing::ShortcutSmoothPath;
using common_robotics_utilities::utility::UniformUnitRealFunction;

template <typename StateType>
std::vector<StateType> PathProcessor<StateType>::ProcessPath(
    const std::vector<StateType>& path, const Parameters& parameters,
    const PlanningSpace<StateType>& planning_space) {
  DRAKE_THROW_UNLESS(parameters.max_smoothing_iterations >= 0);
  DRAKE_THROW_UNLESS(parameters.max_failed_smoothing_iterations >= 0);
  DRAKE_THROW_UNLESS(parameters.max_backtracking_steps >= 0);
  DRAKE_THROW_UNLESS(parameters.max_smoothing_shortcut_fraction >= 0.0);
  DRAKE_THROW_UNLESS(parameters.resampled_state_interval >= 0.0);

  // Bind helper functions.
  const std::function<bool(const StateType&, const StateType&)>
      edge_validity_check_fn = [&](const StateType& q1, const StateType& q2) {
        return planning_space.CheckEdgeValidity(q1, q2);
      };

  const std::function<double(const StateType&, const StateType&)>
      state_distance_fn = [&](const StateType& q1, const StateType& q2) {
        return planning_space.StateDistanceForwards(q1, q2);
      };

  const std::function<StateType(const StateType&, const StateType&,
                                const double)>
      state_interpolation_fn =
          [&](const StateType& q1, const StateType& q2, const double ratio) {
            return planning_space.InterpolateForwards(q1, q2, ratio);
          };

  // Optional safety check.
  if (parameters.safety_check_path) {
    DRAKE_THROW_UNLESS(planning_space.CheckPathValidity(path));
  }

  // Do postprocessing.
  std::vector<StateType> processed_path = path;
  if (parameters.resample_before_smoothing ||
      (!parameters.use_shortcut_smoothing &&
       parameters.resample_after_smoothing)) {
    drake::log()->log(parameters.processor_log_level,
                      "Calling pre-smoothing ResamplePath()...");
    const size_t starting_length = processed_path.size();
    processed_path =
        ResamplePath(processed_path, parameters.resampled_state_interval,
                     state_distance_fn, state_interpolation_fn);
    drake::log()->log(parameters.processor_log_level,
                      "Resampled path from {} to {} states", starting_length,
                      processed_path.size());
  }

  if (parameters.use_shortcut_smoothing) {
    drake::log()->log(parameters.processor_log_level,
                      "Calling ShortcutSmoothPath()...");
    std::mt19937_64 prng(parameters.prng_seed);
    const UniformUnitRealFunction uniform_unit_real_fn = [&]() {
      return DrawUniformUnitReal(&prng);
    };

    const size_t starting_length = processed_path.size();
    const double resampling_interval = (parameters.resample_shortcuts)
                                           ? parameters.resampled_state_interval
                                           : 0.0;
    const std::chrono::time_point<std::chrono::steady_clock> start_time =
        std::chrono::steady_clock::now();
    processed_path = ShortcutSmoothPath(
        processed_path,
        static_cast<uint32_t>(parameters.max_smoothing_iterations),
        static_cast<uint32_t>(parameters.max_failed_smoothing_iterations),
        static_cast<uint32_t>(parameters.max_backtracking_steps),
        parameters.max_smoothing_shortcut_fraction, resampling_interval,
        parameters.check_for_marginal_shortcuts, edge_validity_check_fn,
        state_distance_fn, state_interpolation_fn, uniform_unit_real_fn);
    const std::chrono::time_point<std::chrono::steady_clock> end_time =
        std::chrono::steady_clock::now();
    const std::chrono::duration<double> smoothing_time = end_time - start_time;
    drake::log()->log(parameters.processor_log_level,
                      "Shortcut path from {} to {} states in {} seconds",
                      starting_length, processed_path.size(),
                      smoothing_time.count());
  }

  if (parameters.use_shortcut_smoothing &&
      parameters.resample_after_smoothing) {
    drake::log()->log(parameters.processor_log_level,
                      "Calling post-smoothing ResamplePath()...");
    const size_t starting_length = processed_path.size();
    processed_path =
        ResamplePath(processed_path, parameters.resampled_state_interval,
                     state_distance_fn, state_interpolation_fn);
    drake::log()->log(parameters.processor_log_level,
                      "Resampled path from {} to {} states", starting_length,
                      processed_path.size());
  }
  drake::log()->log(parameters.processor_log_level, "Postprocessing complete");

  // Optional safety check.
  if (parameters.safety_check_path) {
    DRAKE_THROW_UNLESS(planning_space.CheckPathValidity(processed_path));
  }
  return processed_path;
}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::PathProcessor)
