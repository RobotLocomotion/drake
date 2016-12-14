#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
class System;

constexpr int kAutoSize = -1;

// TODO(david-german-tri): Create separate InputPortDescriptor and
// OutputPortDescriptor, then get rid of this enum.
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
  ///              are indexed separately. They both start from zero and
  ///              increment by one per port.
  /// @param data_type Whether the port described is vector or abstract valued.
  /// @param size If the port described is vector-valued, the number of
  ///             elements, or kAutoSize if determined by connections.
  SystemPortDescriptor(const System<T>* system, PortFaceType face, int index,
                       PortDataType data_type, int size)
      : system_(system),
        face_(face),
        index_(index),
        data_type_(data_type),
        size_(size) {}
  virtual ~SystemPortDescriptor() {}

  const System<T>* get_system() const { return system_; }
  PortFaceType get_face() const { return face_; }
  int get_index() const { return index_; }
  PortDataType get_data_type() const { return data_type_; }
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
  const PortDataType data_type_;
  const int size_;
};

}  // namespace systems
}  // namespace drake
