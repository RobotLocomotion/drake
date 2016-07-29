#pragma once

namespace drake {
namespace systems {
namespace plants {

/**
 * Defines the type of a model element.
 */
enum ModelElementType {
  kBodyElement = 0,
  kJointElement,
  kForceElement,
  kSensorElement,
  kFrameElement,
  kLoopElement,
  kActuatorElement
};

}  // namespace plants
}  // namespace systems
}  // namespace drake
