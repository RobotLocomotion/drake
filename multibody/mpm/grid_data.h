#pragma once

#include <cstdint>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* This class is a lightweight wrapper around an integer type (`int32_t` or
 `int64_t`) used to differentiate between active grid node indices, inactive
 states, and a special flag. An IndexOrFlag can be in exactly one of the
 following states:

  1. Active index: A non-negative integer representing the index of a grid node.
  2. Inactive state (the default state): neither an index nor a flag.
  3. The `flag` state: A special state used to mark grid nodes for deferred
     processing. The flag state is intended as a marker that must only be set
     from the inactive state (not from an active index) to maintain consistency.

 Transitions between states are as follows:
  - Any state can become inactive.
  - The inactive state can become flag state.
  - The inactive state can become active (with a non-negative index).
  - Active cannot directly become flag state (must go inactive first).

 An IndexOrFlag object is guaranteed to have size equal to its template
 parameter T. In addition, the inactive state is guaranteed to be represented
 with the value -1.

 @tparam T The integer type for the index. Must be `int32_t` or `int64_t`. */
template <typename T>
class IndexOrFlag {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IndexOrFlag);

  static_assert(std::is_same_v<T, int32_t> || std::is_same_v<T, int64_t>,
                "T must be int32_t or int64_t.");

  /* Default constructor initializes the object to the inactive state. */
  constexpr IndexOrFlag() = default;

  /* Constructor for an active index.
   @pre index >= 0 */
  explicit constexpr IndexOrFlag(T index) { set_index(index); }

  /* Sets the index to the given value, which must be non-negative, thereby
   making `this` active.
   @pre index >= 0 */
  void set_index(T index) {
    DRAKE_ASSERT(index >= 0);
    value_ = index;
  }

  /* Sets `this` to the inactive state. */
  void set_inactive() { value_ = kInactive; }

  /* Sets `this` to the flag state.
   @pre !is_index() (i.e., must currently be inactive) */
  void set_flag() {
    DRAKE_ASSERT(!is_index());
    value_ = kFlag;
  }

  /* Returns true iff `this` is an active index (i.e., a non-negative integer).
   */
  constexpr bool is_index() const { return value_ >= 0; }

  /* Returns true iff `this` is in the inactive state. */
  constexpr bool is_inactive() const { return value_ == kInactive; }

  /* Returns true iff `this` is in the flag state. */
  constexpr bool is_flag() const { return value_ == kFlag; }

  /* Returns the index value.
   @pre is_index() == true; */
  constexpr T index() const {
    DRAKE_ASSERT(is_index());
    return value_;
  }

  bool operator==(const IndexOrFlag&) const = default;

 private:
  /* Note that the enum values are not arbitrary; kInactive must be -1 as in the
   class documentation. */
  enum : T { kInactive = -1, kFlag = -2 };
  /* We encode all information in `value_` to satisfy the size requirement laid
   out in the class documentation. */
  T value_{kInactive};
};

/* GridData stores data at a single grid node of SparseGrid.

 It contains the mass and velocity of the node along with a scratch space for
 temporary storage and an index to the node.

 It's important to be conscious of the size of GridData since the MPM algorithm
 is usually memory-bound. We carefully pack GridData to be a power of 2 to work
 with SPGrid, which automatically packs the data to a power of 2.

 @tparam T double or float or AutoDiffXd. AutoDiffXd is for testing only and
 isn't subject to the "power of 2" requirement. */
template <typename T>
struct GridData {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double> ||
                    std::is_same_v<T, AutoDiffXd>,
                "T must be float or double or AutoDiffXd.");

  /* Resets all floating point data to zero and the index to be inactive.*/
  void reset() { *this = {}; }

  /* Returns true iff `this` GridData is bit-wise equal to `other`. */
  bool operator==(const GridData<T>& other) const {
    if constexpr (std::is_floating_point_v<T>) {
      return std::memcmp(this, &other, sizeof(GridData<T>)) == 0;
    } else {
      return std::tie(v, m, scratch, index_or_flag) ==
             std::tie(other.v, other.m, other.scratch, other.index_or_flag);
    }
  }

  /* Returns true iff `this` GridData is inactive. A grid node inactive when
   it's not supported (see SparseGrid). */
  bool is_inactive() const { return index_or_flag.is_inactive(); }

  Vector3<T> v{Vector3<T>::Zero()};
  T m{0};
  Vector3<T> scratch{Vector3<T>::Zero()};
  std::conditional_t<std::is_same_v<T, float>, IndexOrFlag<int32_t>,
                     IndexOrFlag<int64_t>>
      index_or_flag{};
};

/* With T = float, GridData is expected to be 32 bytes. With T = double,
 GridData is expected to be 64 bytes. We enforce these sizes at compile time
 with static_assert, so that if future changes to this code, compiler alignment,
 or Eigen alignment rules cause a size shift, it will be caught early. */
static_assert(sizeof(GridData<float>) == 32,
              "Unexpected size for GridData<float>.");
static_assert(sizeof(GridData<double>) == 64,
              "Unexpected size for GridData<double>.");

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
