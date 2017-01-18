#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
class System;

constexpr int kAutoSize = -1;

/// All system ports are either vectors of Eigen scalars, or black-box
/// AbstractValues which may contain any type.
typedef enum {
  kVectorValued = 0,
  kAbstractValued = 1,
} PortDataType;

/// InputPortDescriptor is a notation for specifying the kind of input a
/// System accepts, on a given port. It is not a mechanism for handling any
/// actual input data.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class InputPortDescriptor {
 public:
  /// @param system The system to which this descriptor belongs.
  /// @param index The index of the input port described, starting from zero and
  ///              incrementing by one per port.
  /// @param data_type Whether the port described is vector or abstract valued.
  /// @param size If the port described is vector-valued, the number of
  ///             elements, or kAutoSize if determined by connections.
  InputPortDescriptor(const System<T>* system, int index,
                      PortDataType data_type, int size)
      : system_(system), index_(index), data_type_(data_type), size_(size) {
    if (size_ == kAutoSize) {
      DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
    }
  }

  /// @name Basic Concepts
  /// MoveConstructible, CopyConstructible, Copy-Assignable, Move-Assignable
  /// @{
  InputPortDescriptor() = delete;
  InputPortDescriptor(InputPortDescriptor&& other) = default;
  InputPortDescriptor(const InputPortDescriptor&) = default;
  InputPortDescriptor& operator=(InputPortDescriptor&&) = default;
  InputPortDescriptor& operator=(const InputPortDescriptor&) = default;
  ~InputPortDescriptor() = default;
  /// @}

  const System<T>* get_system() const { return system_; }
  int get_index() const { return index_; }
  PortDataType get_data_type() const { return data_type_; }
  int size() const { return size_; }

 private:
  const System<T>* system_{nullptr};
  int index_{0};
  PortDataType data_type_{kVectorValued};
  int size_{0};
};

/// OutputPortDescriptor is a notation for specifying the kind of output a
/// System produces, on a given port. It is not a mechanism for handling any
/// actual output data.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class OutputPortDescriptor {
 public:
  /// @param system The system to which this descriptor belongs.
  /// @param index The index of the output port described, starting from zero
  ///              and incrementing by one per port.
  /// @param data_type Whether the port described is vector or abstract valued.
  /// @param size If the port described is vector-valued, the number of
  ///             elements, or kAutoSize if determined by connections.
  OutputPortDescriptor(const System<T>* system, int index,
                       PortDataType data_type, int size)
      : system_(system), index_(index), data_type_(data_type), size_(size) {
    if (size_ == kAutoSize) {
      DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
    }
  }
  /// @name Basic Concepts
  /// MoveConstructible, CopyConstructible, Copy-Assignable, Move-Assignable
  /// @{
  OutputPortDescriptor() = delete;
  OutputPortDescriptor(OutputPortDescriptor&&) = default;
  OutputPortDescriptor(const OutputPortDescriptor&) = default;
  OutputPortDescriptor& operator=(OutputPortDescriptor&&) = default;
  OutputPortDescriptor& operator=(const OutputPortDescriptor&) = default;
  ~OutputPortDescriptor() = default;
  /// @}

  const System<T>* get_system() const { return system_; }
  int get_index() const { return index_; }
  PortDataType get_data_type() const { return data_type_; }
  int size() const { return size_; }

 private:
  const System<T>* system_{nullptr};
  int index_{0};
  PortDataType data_type_{kVectorValued};
  int size_{0};
};

}  // namespace systems
}  // namespace drake
