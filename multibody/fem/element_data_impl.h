#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* ElementDataImpl implements ElementData for a particular type of FEM element.
 It is templated on the concrete FemElement type in order to allow compile time
 optimizations based on fixed sizes.
 @tparam Element The type of FemElement that consumes this ElementData. This
 template parameter provides the scalar type and the type of per-element data
 this FemStateImpl stores. */
template <typename Element>
class ElementDataImpl : public ElementData<typename Element::T> {
 public:
  using T = typename Element::T;
  using Data = typename Element::Traits::Data;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ElementDataImpl);

  ElementData(int num_elements) : element_data_(num_elements) {}

  int size() const final { return element_data_.size(); }

  void set_data(int index, Data data) {
    DRAKE_ASSERT(0 <= index && index < size());
    element_data_[index] = std::move(data);
  }

  const Data& get_data(int index) {
    DRAKE_ASSERT(0 <= index && index < size());
    return element_data_[index];
  }

 private:
  std::vector<Element::Traits::Data> element_data_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
