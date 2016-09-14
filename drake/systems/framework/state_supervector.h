#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// StateSupervector is a concrete class template that implements
/// VectorBase by concatenating multiple VectorBases, which it
/// does not own.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class StateSupervector : public VectorBase<T> {
 public:
  /// Constructs a supervector consisting of all the vectors in
  /// subvectors, which must live at least as long as this supervector.
  explicit StateSupervector(const std::vector<VectorBase<T>*>& subvectors)
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

  const T GetAtIndex(int index) const override {
    const auto target = GetSubvectorAndOffset(index);
    return target.first->GetAtIndex(target.second);
  }

  void SetAtIndex(int index, const T& value) override {
    const auto target = GetSubvectorAndOffset(index);
    target.first->SetAtIndex(target.second, value);
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
                              " out of bounds for state supervector of size " +
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

  // StateSupervector objects are neither copyable nor moveable.
  StateSupervector(const StateSupervector& other) = delete;
  StateSupervector& operator=(const StateSupervector& other) = delete;
  StateSupervector(StateSupervector&& other) = delete;
  StateSupervector& operator=(StateSupervector&& other) = delete;

  // An ordered list of all the constituent vectors in this supervector.
  std::vector<VectorBase<T>*> vectors_;

  // The integer in the lookup_table_ at index N is the sum of the number of
  // state variables in the constituent vectors 0 through N inclusive.
  // For example, if the lengths of the constituent vectors are [1, 3, 5],
  // the lookup table is [1, 4, 9].
  std::vector<int> lookup_table_;
};

}  // namespace systems
}  // namespace drake
