#pragma once

#include <cstdint>
#include <vector>

#include <spdlog/spdlog.h>

#include "drake/planning/sampling_based/dev/default_state_types.h"
#include "drake/planning/sampling_based/dev/planning_space.h"

namespace drake {
namespace planning {
/// Path processor for performing resampling and shortcut smoothing.
/// Note: without using a continuous collision checker, there is no guarantee
/// that an arbitrary resampling of a collision-free path will remain
/// collision-free. Generally, these "marginal" collision may be ignored,
/// particularly if collision padding is used to ensure a non-zero nominal
/// distance between robot(s) and obstacles.
template <typename StateType>
class PathProcessor {
 public:
  /// Parameters to the path processor.
  // TODO(calderpg) Provide/document good defaults.
  struct Parameters {
    /// Maximum fraction of path length to attempt to shortcut. Larger values
    /// produce more aggressive smoothing, but will fail to smooth more often.
    double max_smoothing_shortcut_fraction{0.0};
    /// Nominal state-to-state distance to resample the path to.
    double resampled_state_interval{0.0};
    /// Seed for internal random number generator.
    uint64_t prng_seed{0};
    /// Max number of iterations for shortcut smoother.
    int max_smoothing_iterations{0};
    /// Max number of failed shortcut smoothing iterations before the smoother
    /// terminates.
    int max_failed_smoothing_iterations{0};
    /// Max depth to attempt backtracking on unsuccessful shortcuts. In a
    /// backtrack, the candidate shortcut is divided in half and a shortcut is
    /// attempted on each half.
    int max_backtracking_steps{0};
    /// Enable the shortcut smoother.
    bool use_shortcut_smoothing{false};
    /// Resample the path before performing smoothing.
    bool resample_before_smoothing{false};
    /// Resample shortcuts during smoothing.
    bool resample_shortcuts{false};
    /// Resample the path after performing smoothing.
    bool resample_after_smoothing{false};
    /// Check for (and reject) shortcuts that are no longer collision free if
    /// they are resampled.
    bool check_for_marginal_shortcuts{false};
    /// Additional sanity check before and after processing to check that
    /// initial path is valid and processed path remains valid.
    bool safety_check_path{false};
    /// Log level used for the processor's own log messages.
    spdlog::level::level_enum processor_log_level{spdlog::level::debug};
  };

  /// Process the provided path.
  /// @param path Provided path.
  /// @param parameters Parameters to path processor.
  /// @param planning_space Planning space to use.
  /// @return Processed path.
  static std::vector<StateType> ProcessPath(
      const std::vector<StateType>& path, const Parameters& parameters,
      const PlanningSpace<StateType>& planning_space);

  // Delete all constructors of this static-only class.
  PathProcessor(const PathProcessor&) = delete;
};
}  // namespace planning
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::PathProcessor)
