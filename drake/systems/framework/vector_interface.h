#pragma once

#include <cstdint>
#include <memory>

#include "drake/common/eigen_types.h"

#include <Eigen/Dense>

namespace drake {
namespace systems {

/// VectorInterface is a pure abstract interface that real-valued signals
/// between Systems must satisfy. Classes that inherit from VectorInterface
/// may provide names for the elements of the vector, and may also
/// provide other computations for the convenience of Systems handling the
/// signal. The vector is always a column vector with elements stored in
/// contiguous memory locations.
///
/// @tparam T Must be a Scalar compatible with Eigen.
template <typename T>
class VectorInterface {
 public:
  virtual ~VectorInterface() {}

  /// Returns the size of the vector, which must be equal to the number of rows
  /// in get_value().
  virtual int size() const = 0;

  /// Sets the vector to the given value. After a.set_value(b.get_value()), a
  /// must be identical to b.
  /// May throw std::out_of_range if the new value has different dimensions
  /// than expected by the concrete class implementing VectorInterface.
  virtual void set_value(const Eigen::Ref<const VectorX<T>>& value) = 0;

  /// Returns a column vector containing the entire value of the signal.
  virtual Eigen::VectorBlock<const VectorX<T>> get_value() const = 0;

  /// Returns a reference that allows mutation of the values in this vector, but
  /// does not allow resizing the vector itself.
  virtual Eigen::VectorBlock<VectorX<T>> get_mutable_value() = 0;

  /// Copies the entire vector to a new VectorInterface, with the same concrete
  /// implementation type.
  virtual std::unique_ptr<VectorInterface<T>> Clone() const = 0;

 protected:
  VectorInterface() {}

 private:
  // VectorInterface objects are neither copyable nor moveable.
  VectorInterface(const VectorInterface<T>& other) = delete;
  VectorInterface& operator=(const VectorInterface<T>& other) = delete;
  VectorInterface(VectorInterface<T>&& other) = delete;
  VectorInterface& operator=(VectorInterface<T>&& other) = delete;
};

/// This concrete class provides object semantics to an abstract
/// VectorInterface by implementing a copy constructor and copy assignment using
/// the VectorInterface's `Clone()` method. A %VectorObject is 
/// default-constructible, copy-constructible, move-constructible, and
/// copy- and move-assignable.

// TODO(sherm1) This wrapper would not be necessary if we had a smart
// pointer for adding object semantics to abstract objects that support Clone(),
// say copy_unique_ptr<T>. Consider adapting Simbody's ClonePtr<T>.
template <typename T>
class VectorObject {
 public:
  /// Constructs an empty %VectorObject.
  VectorObject() noexcept {}

  /// Takes over ownership of the provided VectorInterface object.
  explicit VectorObject(std::unique_ptr<VectorInterface<T>> vector) noexcept
      : vector_(std::move(vector)) {}

  /// Copy constructor uses the `source` object's `Clone()` method to make
  /// a deep copy which is then owned by this %VectorObject.
  VectorObject(const VectorObject& source) : VectorObject() {
    if (!source.empty())
      vector_.reset(source.get_vector().Clone());
  }

  /// Move construction leaves `source` empty.
  VectorObject(VectorObject&& source) noexcept
    : vector_(std::move(source.vector_)) {}

  /// Copy assignment replaces the current contents of this %VectorObject with
  /// a clone of the `source` object.
  VectorObject& operator=(const VectorObject& source) {
    vector_.reset();  // Delete current vector.
    if (!source.empty()) vector_.reset(source.get_vector().Clone());
    return *this;
  }

  /// Move assignment leaves `source` empty.
  VectorObject& operator=(VectorObject&& source) noexcept {
    vector_ = std::move(source.vector_);
    return *this;
  }

  /// Returns a const reference to the VectorInterface object owned by this
  /// %VectorObject, if any. Don't call this on an empty object; check first
  /// with empty() if you aren't sure.
  /// @throws std::logic_error This object is empty.
  const VectorInterface& get_vector() const {
    if (empty())
      throw std::logic_error("VectorObject::get_vector(): object is empty.");
    return *vector_;
  }

  /// Returns a mutable pointer to the VectorInterface object owned by this
  /// %VectorObject, if any. Don't call this on an empty object; check first
  /// with empty() if you aren't sure.
  /// @throws std::logic_error This object is empty.
  VectorInterface* get_mutable_vector() {
    if (empty()) {
      throw std::logic_error(
          "VectorObject::get_mutable_vector(): object is empty.");
    }
    return vector_.get();
  }

  /// Returns `true` if this %VectorObject does not own a VectorInterface
  /// object. Check this first before calling get_vector() or
  /// get_mutable_vector().
  bool empty() const noexcept { return !vector_; }

 private:
  std::unique_ptr<VectorInterface<T>> vector_;
};

}  // namespace systems
}  // namespace drake
