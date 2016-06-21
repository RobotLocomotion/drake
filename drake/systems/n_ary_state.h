#pragma once

#include <cmath>
#include <stdexcept>
#include <type_traits>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/core/Vector.h"

namespace drake {

using Drake::toEigen;  // TODO(maddog) ...until Drake-->drake fully.

/// NAryState is a Drake::Vector (concept implementation) which is a
/// container of zero or more component Drake::Vector instances.  All
/// components must be of the same type, @p UnitVector, which naturally
/// must model the Drake::Vector concept itself.
///
/// UnitVectors are assembled into NAryState at run-time as an ordered
/// list with O(1) access.  The Eigen::Matrix representaion of a
/// complete NAryState is basically the concatenation of Eigen::Matrix's
/// of the component UnitVectors.
template <class UnitVector>
class NAryState {
 public:
  // The Drake::Vector concept has no explicit way of recovering the
  // ScalarType upon which the template was specialized, but through
  // the miracle of decltype(), it can be done.  std::decay<> is used
  // to strip any CV or reference qualifiers off the type.
  using UnitScalar =
      typename std::decay<decltype(toEigen(UnitVector())(0))>::type;
  // Required by Drake::Vector concept.
  static const int RowsAtCompileTime = Eigen::Dynamic;
  // Required by Drake::Vector concept.
  typedef Eigen::Matrix<UnitScalar, RowsAtCompileTime, 1> EigenType;

  NAryState() : unit_size_(unit_size()), count_(UnitCountFromRows(0)) {}

  explicit NAryState(std::ptrdiff_t count)
      : unit_size_(unit_size()),
        // Ensure correct internal count_ (i.e., -1 if UnitVector's size
        // is zero).
        count_(UnitCountFromRows(RowsFromUnitCount(count))),
        combined_vector_(EigenType::Constant(RowsFromUnitCount(count), 1,
                                             NAN)) {}

  /// @return the count of @param UnitVector units contained within this
  /// NAryState object.
  ///
  /// If UnitVector is a null vector (zero rows), then the count is
  /// indeterminate and the return value is always < 0.
  std::ptrdiff_t count() const { return count_; }

  /// Appends the @param unit to the end of the list of component
  /// @param UnitVectors.
  void Append(const UnitVector& unit) {
    if (unit_size_ == 0) {
      // NOP --- in particular, count_ should remain at -1.
      DRAKE_ASSERT(count_ == -1);
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

  /// @return a copy of the component UnitVector at position @p pos.
  ///
  /// @throws std::out_of_range if UnitVector is a non-NullVector type
  /// and @p pos exceeds the range [0, count()].
  UnitVector get(std::ptrdiff_t pos) const {
    if (unit_size_ > 0) {
      if ((pos < 0) || (pos >= count_)) {
        throw std::out_of_range("Position pos exceeds range [0, count()].");
      }
    }
    const std::size_t row0 = pos * unit_size_;
    return UnitVector(combined_vector_.block(row0, 0,
                                             unit_size_, 1));
  }

  /// Sets the value of the component UnitVector at position @p pos.
  ///
  /// @throws std::out_of_range if UnitVector is a non-NullVector type
  /// and @p pos exceeds the range [0, count()].
  void set(std::ptrdiff_t pos, const UnitVector& unit) {
    if (unit_size_ > 0) {
      if ((pos < 0) || (pos >= count_)) {
        throw std::out_of_range("Position pos exceeds range [0, count()].");
      }
    }
    const std::size_t row0 = pos * unit_size_;
    combined_vector_.block(row0, 0, unit_size_, 1) = toEigen(unit);
  }

  // Required by Drake::Vector concept.
  template <typename Derived>
  // NOLINTNEXTLINE(runtime/explicit)
  NAryState(const Eigen::MatrixBase<Derived>& initial)
      : unit_size_(unit_size()),
        count_(UnitCountFromRows(initial.rows())),
        combined_vector_(initial) {
  }

  // Required by Drake::Vector concept.
  template <typename Derived>
  NAryState& operator=(const Eigen::MatrixBase<Derived>& rhs) {
    count_ = UnitCountFromRows(rhs.rows());
    combined_vector_ = rhs;
    return *this;
  }

  // Required by Drake::Vector concept.
  std::size_t size() const { return combined_vector_.rows(); }

  // Required by Drake::Vector concept.
  friend EigenType toEigen(const NAryState<UnitVector>& vec) {
    return vec.combined_vector_;
  }

  /// Calculates the size (Eigen row count) of @p UnitVector, which is
  /// presumed to be fixed for all instances of UnitVector.
  static
  std::size_t unit_size() { return UnitVector().size(); }

  /// Determines how many @param UnitVector units will be decoded from
  /// an Eigen column matrix with @param rows rows.  @param rows must
  /// be a multiple of the row count of @param UnitVector.
  ///
  /// As a special case, if UnitVector comprises zero rows (i.e., it is
  /// a NullVector), then the return value is always -1.
  ///
  /// @throws std::domain_error if UnitVector is not a null vector and
  /// rows is not a multiple of UnitVector::size().
  static
  std::ptrdiff_t UnitCountFromRows(std::size_t rows) {
    const std::size_t us { unit_size() };
    if (us > 0) {
      if ((rows % us) != 0) {
        throw std::domain_error("Row count not a multiple of non-null unit.");
      }
      return rows / us;
    }
    return -1;
  }

  /// Determines how many Eigen matrix rows will be needed to represent
  /// @param count instances of @param UnitVector.
  ///
  /// To complement UnitCountFromRows(), if @param count is negative,
  /// the return value is zero.  However, @throws std::domain_error if
  /// count is negative and UnitVector is not a null vector.
  static
  std::size_t RowsFromUnitCount(std::ptrdiff_t count) {
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

}  // namespace drake
