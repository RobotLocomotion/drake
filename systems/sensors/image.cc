#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

#if  __cplusplus < 201703L
constexpr float InvalidDepth::kTooFar;
constexpr float InvalidDepth::kTooClose;

constexpr int16_t Label::kNoBody;
constexpr int16_t Label::kFlatTerrain;
#endif

}  // namespace sensors
}  // namespace systems
}  // namespace drake
