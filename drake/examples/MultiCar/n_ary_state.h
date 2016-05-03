#pragma once

#include <Eigen/Dense>


namespace Drake {

template <typename ScalarType,
          template <typename> class UnitVector>
class NAryState {
 public:
  // Required by Drake::Vector concept.
  static const int RowsAtCompileTime = Eigen::Dynamic;

  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  NAryState() : count_(unitCountFromRows(0)) {}

  NAryState(int count)
      : count_(unitCountFromRows(rowsFromUnitCount(count))),
        combined_vector_(EigenType(rowsFromUnitCount(count), 1)) {}

  void append(const UnitVector<ScalarType>& unit) {
    const std::size_t unit_size = UnitVector<ScalarType>::RowsAtCompileTime;
    if (unit_size <= 0) {
      // NOP --- in particular, count_ should remain at -1.
      return;
    }
    // Enlarge combined_vector_ by size of the converted unit.
    combined_vector_.conservativeResize(combined_vector_.rows() + unit_size,
                                        Eigen::NoChange);
    // Copy unit's eigen-rep into the tail-end of enlarged combined_vector_.
    combined_vector_.bottomRows(unit_size) = toEigen(unit);
    // Keep track of the total unit count.
    ++count_;
  }

  UnitVector<ScalarType> get(std::size_t i) const {
    const std::size_t unit_size = UnitVector<ScalarType>::RowsAtCompileTime;
    const std::size_t row0 = i * unit_size;
    return UnitVector<ScalarType>(combined_vector_.block(row0, 0,
                                                         unit_size, 1));
  }

  void set(std::size_t i, const UnitVector<ScalarType>& unit) {
    const std::size_t unit_size = UnitVector<ScalarType>::RowsAtCompileTime;
    const std::size_t row0 = i * unit_size;
    combined_vector_.block(row0, 0, unit_size, 1) = toEigen(unit);
  }

  // Required by Drake::Vector concept.
  template <typename Derived>
  // NOLINTNEXTLINE(runtime/explicit)
  explicit NAryState(const Eigen::MatrixBase<Derived>& initial)
      : count_(unitCountFromRows(initial.rows())),
        combined_vector_(initial) {
  }

  // Required by Drake::Vector concept.
  template <typename Derived>
  NAryState& operator=(const Eigen::MatrixBase<Derived>& rhs) {
    count_ = unitCountFromRows(rhs.rows());
    combined_vector_ = rhs;
    return *this;
  }

  // Required by Drake::Vector concept.
  std::size_t size() const { return combined_vector_.rows(); }

  // Required by Drake::Vector concept.
  friend EigenType toEigen(const NAryState<ScalarType, UnitVector>& vec) {
    return vec.combined_vector_;
  }

  static
  std::size_t unitCountFromRows(std::size_t rows) {
    static_assert(UnitVector<ScalarType>::RowsAtCompileTime != Eigen::Dynamic,
                  "Size of component subvector not known at compile time.");
    if (UnitVector<ScalarType>::RowsAtCompileTime > 0) {
      assert((rows % UnitVector<ScalarType>::RowsAtCompileTime) == 0);
      return rows / UnitVector<ScalarType>::RowsAtCompileTime;
    }
    return -1;
  }

  static
  std::size_t rowsFromUnitCount(std::size_t count) {
    static_assert(UnitVector<ScalarType>::RowsAtCompileTime != Eigen::Dynamic,
                  "Size of component subvector not known at compile time.");
    if (count > 0) {
      return count * UnitVector<ScalarType>::RowsAtCompileTime;
    }
    return 0;
  }

 private:
  std::size_t count_;
  EigenType combined_vector_;
};

} // namespace Drake
