#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/pointer_cast.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

/// A simple subclass of BasicVector<T> for testing, particularly for cases
/// where BasicVector subtyping must be preserved through the framework.
template <typename T, int N>
class MyVector : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyVector)

  /// Constructs an uninitialized N-vector.
  MyVector() : BasicVector<T>(N) {}

  /// Constructs from a variable-length vector whose length must be N.
  explicit MyVector(const VectorX<T>& data) : BasicVector<T>(data) {
    DRAKE_THROW_UNLESS(data.size() == N);
  }

  /// Constructs from a fixed-size Eigen VectorN.
  explicit MyVector(const Eigen::Matrix<T, N, 1>& data)
      : BasicVector<T>(data) {}

  /// Constructs a MyVector where each element is constructed using the
  /// placewise-corresponding member of @p args as the sole constructor
  /// argument.  For instance: `MyVector<2, double>::Make(1.1, 2.2)`.
  template<typename... Fargs>
  static std::unique_ptr<MyVector> Make(Fargs&&... args) {
    static_assert(sizeof...(args) == N,
                  "The number of arguments must match the MyVector size");
    auto data = std::make_unique<MyVector>();
    BasicVector<T>::MakeRecursive(data.get(), 0, args...);
    return data;
  }

  // TODO(jwnimmer-tri) This is extremely dangerous -- the return type of Clone
  // determines template argument for the Value<> that is type-erased into an
  // AbstractValue; we should not pun away from BasicVector, since many methods
  // in the leaf system and context code assumes that BasicVector is what gets
  // type-erased!
  /// Shadows the base class Clone() method to change the return type, so that
  /// this can be used in `copyable_unique_ptr<MyVector>` and `Value<MyVector>`.
  std::unique_ptr<MyVector<T, N>> Clone() const {
    return dynamic_pointer_cast_or_throw<MyVector<T, N>>(
        BasicVector<T>::Clone());
  }

  // Allow unit tests to read/write the underlying MatrixXd directly.
  using BasicVector<T>::values;

 private:
  // BasicVector's Clone() method handles copying the values; DoClone() is
  // only supposed to allocate a vector of the right concrete type and size.
  [[nodiscard]] MyVector* DoClone() const override {
    return new MyVector();
  }
};

using MyVector1d = MyVector<double, 1>;
using MyVector2d = MyVector<double, 2>;
using MyVector3d = MyVector<double, 3>;
using MyVector4d = MyVector<double, 4>;

}  // namespace systems
}  // namespace drake
