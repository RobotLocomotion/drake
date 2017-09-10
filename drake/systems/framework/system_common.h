#pragma once

namespace drake {
namespace systems {

constexpr int kAutoSize = -1;

/// All system ports are either vectors of Eigen scalars, or black-box
/// AbstractValues which may contain any type.
typedef enum {
  kVectorValued = 0,
  kAbstractValued = 1,
} PortDataType;

/// Some input ports may be specifically labeled as "random"; these are
/// intended to model random noise/disturbance inputs to systems and
/// are *often*, but not exclusively, wired up to RandomSource systems.
typedef enum {
  kNotRandom = 0,
  kUniformRandom = 1,   /// Anticipated vector elements are independent and
                        /// uniformly distributed ∈ [0,1].
  kGaussianRandom = 2,  /// Anticipated vector elements are independent and
                        /// drawn from a mean-zero, unit-variance normal
                        /// distribution.
} RandomPortType;

}  // namespace systems
}  // namespace drake
