#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
class System;

constexpr int kAutoSize = -1;
constexpr double kContinuousSample = 0.0;
constexpr double kAutoSample = -1.0;

typedef enum {
  kInputPort = 0,
  kOutputPort = 1,
} PortFaceType;

/// All system ports are either vectors of Eigen scalars, or black-box
/// AbstractValues which may contain any type.
typedef enum {
  kVectorValued = 0,
  kAbstractValued = 1,
} PortDataType;

/// SystemPortDescriptor is a notation for specifying the kind of output a
/// System produces, or the kind of input it accepts, on a given port. It is
/// not a mechanism for handling any actual input or output data.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class SystemPortDescriptor {
 public:
  /// @param system The system to which this descriptor belongs.
  /// @param face Whether an input or output port is described.
  /// @param index The index of the port described. Input and output ports
  ///              are indexed separately.
  /// @param type Whether the port described is vector or abstract valued.
  /// @param size If the port described is vector-valued, the number of
  ///             elements, or kAutoSize if determined by connections.
  /// @param sample_time_sec The rate at which the port described is written
  ///                        or read. kContinuousSample if continuous,
  ///                        kAutoSample if determined by connections.
  SystemPortDescriptor(const System<T>* system, PortFaceType face, int index,
                       PortDataType type, int size, double sample_time_sec)
      : system_(system),
        face_(face),
        index_(index),
        type_(type),
        size_(size),
        sample_time_sec_(sample_time_sec) {}
  virtual ~SystemPortDescriptor() {}

  const System<T>* get_system() const { return system_; }
  PortFaceType get_face() const { return face_; }
  int get_index() const { return index_; }
  PortDataType get_type() const { return type_; }
  double get_sample_time_sec() const { return sample_time_sec_; }
  int get_size() const {
    if (size_ == kAutoSize) {
      DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
    }
    return size_;
  }

 private:
  const System<T>* const system_;
  const PortFaceType face_;
  const int index_;
  const PortDataType type_;
  const int size_;
  const double sample_time_sec_;
};

}  // namespace systems
}  // namespace drake
