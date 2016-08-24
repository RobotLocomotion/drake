#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// BasicStateAndOutputVector is a concrete class template that implements
/// StateVector in a convenient manner for LeafSystem blocks, and implements
/// VectorBase so that it may also be used as an output.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class BasicStateAndOutputVector : public BasicStateVector<T>,
                                  public VectorBase<T> {
 public:
  /// Constructs a BasicStateAndOutputVector of the specified @p size.
  explicit BasicStateAndOutputVector(int size) : BasicStateVector<T>(size) {}

  /// Constructs a BasicStateAndOutputVector with the specified @p data.
  explicit BasicStateAndOutputVector(const std::vector<T>& data)
      : BasicStateVector<T>(data) {}

  /// Constructs a BasicStateAndOutputVector that owns an arbitrary @p vector,
  /// which must not be nullptr.
  explicit BasicStateAndOutputVector(std::unique_ptr<VectorBase<T>> vector)
      : BasicStateVector<T>(std::move(vector)) {}

  // The size() method overrides both BasicStateVector and VectorBase.
  int size() const override { return this->get_wrapped_vector().size(); }

  // These VectorBase overrides merely delegate to the wrapped object.
  void set_value(const Eigen::Ref<const VectorX<T>>& value) override {
    this->get_wrapped_vector().set_value(value);
  }
  Eigen::VectorBlock<const VectorX<T>> get_value() const override {
    return this->get_wrapped_vector().get_value();
  }
  Eigen::VectorBlock<VectorX<T>> get_mutable_value() override {
    return this->get_wrapped_vector().get_mutable_value();
  }

  // This VectorBase override must not delegate, because we need to
  // maintain our class type (BasicStateAndOutputVector) during cloning.
  std::unique_ptr<VectorBase<T>> CloneVector() const override {
    return std::unique_ptr<VectorBase<T>>(DoClone());
  }

 protected:
  BasicStateAndOutputVector(const BasicStateAndOutputVector& other)
      : BasicStateVector<T>(other) {}

  BasicStateAndOutputVector<T>* DoClone() const override {
    return new BasicStateAndOutputVector<T>(*this);
  }

 private:
  // Disable these, for consistency with parent class.
  BasicStateAndOutputVector& operator=(const BasicStateAndOutputVector&) =
      delete;
  BasicStateAndOutputVector(BasicStateAndOutputVector&&) = delete;
  BasicStateAndOutputVector& operator=(BasicStateAndOutputVector&&) = delete;
};

}  // namespace systems
}  // namespace drake
