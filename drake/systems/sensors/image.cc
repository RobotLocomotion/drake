#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

constexpr float InvalidDepth::kTooFar;
constexpr float InvalidDepth::kTooClose;

constexpr int16_t Label::kNoBody;
constexpr int16_t Label::kFlatTerrain;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
