#pragma once

#include <stdexcept>

#include <Eigen/Dense>

#include "drake/core/Vector.h"

namespace drake {
using Drake::toEigen; // TODO maddog@tri.global  ...until Drake-->drake fully.

template <typename ScalarType,
          template <typename> class UnitVector>
class NAryState {
 public:
  // Required by Drake::Vector concept.
  static const int RowsAtCompileTime = Eigen::Dynamic;

  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  NAryState() : unit_size_(unit_size()), count_(unitCountFromRows(0)) {}

  explicit NAryState(int count)
      : unit_size_(unit_size()),
        count_(unitCountFromRows(rowsFromUnitCount(count))),
        combined_vector_(EigenType(rowsFromUnitCount(count), 1)) {}

  /// Return the count of @param UnitVector units contained.
  ///
  /// If UnitVector is a null vector (zero rows), then the count is
  /// indeterminate and the return value is always < 0.
  std::ptrdiff_t count() const { return count_; }

  /// Append the @param unit to the end of the list of component
  /// @param UnitVectors.
  void append(const UnitVector<ScalarType>& unit) {
    if (unit_size_ == 0) {
      // NOP --- in particular, count_ should remain at -1.
      assert(count_ == -1);
      return;
    }
    // Enlarge combined_vector_ by size of the converted unit.
    combined_vector_.conservativeResize(combined_vector_.rows() + unit_size_,
                                        Eigen::NoChange);
    // Copy unit's Eigen-rep into the tail-end of enlarged combined_vector_.
    combined_vector_.bottomRows(unit_size_) = toEigen(unit);
    // Keep track of the total unit count.
    ++count_;
  }

  UnitVector<ScalarType> get(std::size_t i) const {
    if (!((unit_size_ == 0) || (i < count_))) {
      throw std::out_of_range("");
    }
    const std::size_t row0 = i * unit_size_;
    return UnitVector<ScalarType>(combined_vector_.block(row0, 0,
                                                         unit_size_, 1));
  }

  void set(std::size_t i, const UnitVector<ScalarType>& unit) {
    if (!((unit_size_ == 0) || (i < count_))) {
      throw std::out_of_range("");
    }
    const std::size_t row0 = i * unit_size_;
    combined_vector_.block(row0, 0, unit_size_, 1) = toEigen(unit);
  }

  // Required by Drake::Vector concept.
  template <typename Derived>
  // NOLINTNEXTLINE(runtime/explicit)
  explicit NAryState(const Eigen::MatrixBase<Derived>& initial)
      : unit_size_(unit_size()),
        count_(unitCountFromRows(initial.rows())),
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

  /// Calculate the size (Eigen row count) of @p UnitVector, which is
  /// presumed to be fixed for all instances of UnitVector.
  static
  std::size_t unit_size() { return UnitVector<ScalarType>().size(); }

  /// Determine how many @param UnitVector units will be decoded from
  /// an Eigen column matrix with @param rows rows.  @param rows must
  /// be a multiple of the row count of @param UnitVector.
  ///
  /// As a special case, if UnitVector comprises zero rows (i.e., it is
  /// a NullVector), then the return value is always -1.
  ///
  /// @throws std::domain_error if UnitVector is not a null vector and
  /// rows is not a multiple of UnitVector::RowsAtCompileTime.
  static
  std::ptrdiff_t unitCountFromRows(std::size_t rows) {
    const std::size_t us { unit_size() };
    if (us > 0) {
      if ((rows % us) != 0) {
        throw std::domain_error("Row count not a multiple of non-null unit.");
      }
      return rows / us;
    }
    return -1;
  }

  /// Determine how many Eigen matrix rows will be needed to represent
  /// @param count instances of @param UnitVector.
  ///
  /// To complement unitCountFromRows(), if @param count is negative,
  /// the return value is zero.  However, @throws std::domain_error if
  /// count is negative and UnitVector is not a null vector.
  static
  std::size_t rowsFromUnitCount(std::ptrdiff_t count) {
    const std::size_t us { unit_size() };
    if (count >= 0) {
      return count * us;
    }
    if (us != 0) {
      throw std::domain_error("Negative count for non-null unit.");
    }
    return 0;
  }

 private:
  std::size_t unit_size_;
  // count_ < 0 indicates "not counted", i.e., UnitVector is a null vector.
  std::ptrdiff_t count_;
  EigenType combined_vector_;
};

} // namespace Drake
