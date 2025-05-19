
#include <cstddef>
#include <iterator>
#include <vector>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
class VectorView {
 public:
  using value_type = T;
  using pointer = T*;
  using reference = T&;
  using const_reference = const T&;
  using iterator = T*;
  using const_iterator = const T*;
  using size_type = std::size_t;

  VectorView() : data_(nullptr), size_(0) {}

  VectorView(T* data, size_type size) : data_(data), size_(size) {}

  VectorView(std::vector<T>& vec, size_type start, size_type end) {
    assert(start <= end && end <= vec.size());
    data_ = vec.data() + start;
    size_ = end - start;
  }

  reference operator[](size_type index) {
    assert(index < size_);
    return data_[index];
  }

  const_reference operator[](size_type index) const {
    assert(index < size_);
    return data_[index];
  }

  iterator begin() { return data_; }
  iterator end() { return data_ + size_; }

  const_iterator begin() const { return data_; }
  const_iterator end() const { return data_ + size_; }

  const_iterator cbegin() const { return data_; }
  const_iterator cend() const { return data_ + size_; }

  size_type size() const { return size_; }
  bool empty() const { return size_ == 0; }

 private:
  T* data_;
  size_type size_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
