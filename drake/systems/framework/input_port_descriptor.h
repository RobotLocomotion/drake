#pragma once

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/system_common.h"

namespace drake {
namespace systems {

template <typename T>
class System;

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
  /// MoveConstructible only; not CopyConstructible; not Copy/Move-Assignable.
  /// @{
  //
  // Implementation note: This class aliases a pointer to the system that
  // contains it and captures its own index within that system's port vector,
  // so we must be careful not to allow C++ copying to extend the lifetime of
  // the system alias or duplicate our claim to the index.  Thus, this class is
  // `MoveConstructible` but neither copyable nor assignable: it supports move
  // to populate a vector, but is non-copyable in order remain the "one true
  // descriptor" after construction and non-assignable in order to remain
  // const.  Code that wishes to refer to this descriptor after insertion into
  // the vector should use a reference (not copy) of this descriptor.
  InputPortDescriptor() = delete;
  InputPortDescriptor(InputPortDescriptor&& other) = default;
  InputPortDescriptor(const InputPortDescriptor&) = delete;
  InputPortDescriptor& operator=(InputPortDescriptor&&) = delete;
  InputPortDescriptor& operator=(const InputPortDescriptor&) = delete;
  ~InputPortDescriptor() = default;
  /// @}

  const System<T>* get_system() const { return system_; }
  int get_index() const { return index_; }
  PortDataType get_data_type() const { return data_type_; }
  int size() const { return size_; }

 private:
  const System<T>* const system_;
  const int index_;
  const PortDataType data_type_;
  const int size_;
};

}  // namespace systems
}  // namespace drake
