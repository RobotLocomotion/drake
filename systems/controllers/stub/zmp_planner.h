#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/planning/locomotion/zmp_planner.h"

namespace drake {
namespace systems {
namespace controllers {

using ZmpPlanner DRAKE_DEPRECATED("2024-08-01",
    "Use drake/planning/locomotion/zmp_planner.h instead.")
    = drake::planning::ZmpPlanner;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
