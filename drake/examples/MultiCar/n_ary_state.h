#pragma once

#include <Eigen/Dense>


namespace Drake {

template <typename ScalarType,
          template <typename> class UnitVector>
///template <template <typename> class UnitVector>
///class WRAPPO {
/// public:
///template <typename ScalarType>
class NAryState {
 public:
  NAryState() {
    static_assert(UnitVector<ScalarType>::RowsAtCompileTime != Eigen::Dynamic,
                  "Size of component subvector not known at compile time.");
  }

  void append(const UnitVector<ScalarType>& unit) {
    const std::size_t unit_size = UnitVector<ScalarType>::RowsAtCompileTime;
    // Enlarge combined_vector_ by size of the converted unit.
    combined_vector_.conservativeResize(combined_vector_.rows() + unit_size,
                                        Eigen::NoChange);
    // Copy unit's eigen-rep into the tail-end of our combined_vector_.
    combined_vector_.bottomRows(unit_size) = toEigen(unit);
  }

  UnitVector<ScalarType> get(std::size_t i) const {
    const std::size_t unit_size = UnitVector<ScalarType>::RowsAtCompileTime;
    const std::size_t row0 = i * unit_size;
    return UnitVector<ScalarType>(
        combined_vector_.block(row0, 0, unit_size, 1));
  }

  // Required by Drake::Vector concept.
  template <typename Derived>
  // NOLINTNEXTLINE(runtime/explicit)
  NAryState(const Eigen::MatrixBase<Derived>& initial)
      : combined_vector_(initial) {
    static_assert(UnitVector<ScalarType>::RowsAtCompileTime != Eigen::Dynamic,
                  "Size of component subvector not known at compile time.");
    // TODO maddog@tri.global Assert that initial.size() is multiple
    //                        of UnitVector.
  }

  // Required by Drake::Vector concept.
  template <typename Derived>
  NAryState& operator=(const Eigen::MatrixBase<Derived>& rhs) {
    // TODO maddog@tri.global Assert that rhs.size() is multiple of UnitVector.
    combined_vector_ = rhs;
    return *this;
  }

  // Required by Drake::Vector concept.
  static const int RowsAtCompileTime = Eigen::Dynamic;
#if 0
  enum {
    RowsAtCompileTime =
    (UnitVector<ScalarType>::RowsAtCompileTime == 0) ? 0 : Eigen::Dynamic
  };
#endif

  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  // Required by Drake::Vector concept.
  std::size_t size() const { return combined_vector_.rows(); }

  // Required by Drake::Vector concept.
  friend EigenType toEigen(const NAryState<ScalarType, UnitVector>& vec) {
    return vec.combined_vector_;
  }

 private:
  EigenType combined_vector_;
};

} // namespace Drake
