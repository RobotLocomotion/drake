#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// Supervector is a concrete class template that implements
/// VectorBase by concatenating multiple VectorBases, which it
/// does not own.
///
/// @tparam_default_scalar
template <typename T>
class Supervector : public VectorBase<T> {
 public:
  // Supervector objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Supervector)

  /// Constructs a supervector consisting of all the vectors in
  /// subvectors, which must live at least as long as this supervector.
  explicit Supervector(const std::vector<VectorBase<T>*>& subvectors)
      : vectors_(subvectors) {
    int sum = 0;
    for (const VectorBase<T>* vec : vectors_) {
      sum += vec->size();
      lookup_table_.push_back(sum);
    }
  }

  int size() const override {
    return lookup_table_.empty() ? 0 : lookup_table_.back();
  }

 protected:
  const T& DoGetAtIndex(int index) const override {
    const auto target = GetSubvectorAndOffset(index);
    return (*target.first)[target.second];
  }

  T& DoGetAtIndex(int index) override {
    const auto target = GetSubvectorAndOffset(index);
    return (*target.first)[target.second];
  }

 private:
  // Given an index into the supervector, returns the subvector that
  // contains that index, and its offset within the subvector. This operation
  // is O(log(N)) in the number of subvectors. Throws std::out_of_range for
  // invalid indices.
  //
  // Example: if the lookup table is [1, 4, 9], and @p index is 5, this
  // function returns a pointer to the third of three subvectors, with offset
  // 1, because the element at index 5 in the supervector is at index 1 in
  // that subvector.
  //
  // 0 | 1 2 3 | 4 5 6 7 8
  //               ^ index 5
  std::pair<VectorBase<T>*, int> GetSubvectorAndOffset(int index) const {
    if (index >= size() || index < 0) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              " out of bounds for supervector of size " +
                              std::to_string(size()));
    }
    // Binary-search the lookup_table_ for the first element that is larger
    // than the specified index.
    const auto it =
        std::upper_bound(lookup_table_.begin(), lookup_table_.end(), index);

    // Use the lookup result to identify the subvector that contains the index.
    const int subvector_id =
        static_cast<int>(std::distance(lookup_table_.begin(), it));
    VectorBase<T>* subvector = vectors_[subvector_id];

    // The item at index 0 in vectors_[subvector_id] corresponds to index
    // lookup_table_[subvector_id - 1] in the supervector.
    const int start_of_subvector = (subvector_id == 0) ? 0 : *(it - 1);
    return std::make_pair(subvector, index - start_of_subvector);
  }

  // An ordered list of all the constituent vectors in this supervector.
  std::vector<VectorBase<T>*> vectors_;

  // The integer in the lookup_table_ at index N is the sum of the number of
  // elements in the constituent vectors 0 through N inclusive.
  // For example, if the sizes of the constituent vectors are [1, 3, 5],
  // the lookup table is [1, 4, 9].
  std::vector<int> lookup_table_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Supervector)
