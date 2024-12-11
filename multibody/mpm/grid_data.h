#pragma once

#include <cstdint>

#include "drake/common/bit_cast.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* This class is a lightweight wrapper around an integer type (`int32_t` or
 `int64_t`) used to differentiate between active indices, inactive states, and
 special flags. A GridNodeIndex can be in exactly one of the following states:

  1. Active index: A non-negative integer representing the index of a grid.
  2. Inactive state (the default state).
  3. The participating state: A special state used to mark grid nodes for
     deferred processing. The participating state is intended as a marker that
     must only be set from the inactive state (not from an active index) to
     maintain consistency.

 Transitions between states are as follows:
  - Any state can become inactive.
  - The inactive state can become participating.
  - The inactive state can become active (with a non-negative index).
  - Active cannot directly become participating (must go inactive first).

 A GridNodeIndex object is guaranteed to have size equal to its template
 parameter T. In addition, the inactive state is guaranteed to be represented
 with the value -1.

 @tparam T The integer type for the index. Must be `int32_t` or `int64_t`. */
template <typename T>
class GridNodeIndex {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GridNodeIndex);

  static_assert(std::is_same_v<T, int32_t> || std::is_same_v<T, int64_t>,
                "T must be int32_t or int64_t.");

  /* Default constructor initializes the index to the inactive state. */
  constexpr GridNodeIndex() = default;

  /* Constructor for an active index.
   @pre index >= 0 */
  explicit constexpr GridNodeIndex(T index) { set_value(index); }

  /* Returns true iff `this` and `other` are bit-wise identical. */
  bool operator==(const GridNodeIndex<T>& other) const = default;

  /* Sets the index to the given value, which must be non-negative, thereby
   making `this` active.
   @pre index >= 0 */
  void set_value(T index) {
    DRAKE_ASSERT(index >= 0);
    value_ = index;
  }

  /* Sets `this` to the inactive state. */
  void set_inactive() { value_ = kInactive; }

  /* Sets `this` to the participating state.
   @pre !is_index() (i.e., must currently be inactive) */
  void set_participating() {
    DRAKE_ASSERT(!is_index());
    value_ = kParticipating;
  }

  /* Returns true iff the index is active (i.e., a non-negative integer). */
  constexpr bool is_index() const { return value_ >= 0; }

  /* Returns true iff the index is in the inactive state. */
  constexpr bool is_inactive() const { return value_ == kInactive; }

  /* Returns true iff the index is in the participating state. */
  constexpr bool is_participating() const { return value_ == kParticipating; }

  /* Returns the index value.
   @pre is_index() == true; */
  constexpr T value() const {
    DRAKE_ASSERT(is_index());
    return value_;
  }

 private:
  enum : T { kInactive = -1, kParticipating = -2 };
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

 @tparam T double or float. */
template <typename T>
struct GridData {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "T must be float or double.");
  using IntType = typename std::conditional<std::is_same_v<T, float>, int32_t,
                                            int64_t>::type;
  static constexpr IntType kAllBitsOn = -1;

  /* Returns a floating point NaN value with all bits set to one. This choice
   makes the reset() function more efficient. */
  static T nan() { return drake::internal::bit_cast<T>(kAllBitsOn); }

  /* Resets `this` GridData to its default state where all floating point values
   are set to NAN and the index is inactive. */
  void reset() {
    v = Vector3<T>::Constant(nan());
    m = nan();
    scratch = Vector3<T>::Constant(nan());
    index.set_inactive();
  }

  /* Returns true iff `this` GridData is bit-wise equal to `other`. */
  bool operator==(const GridData<T>& other) const {
    return std::memcmp(this, &other, sizeof(GridData<T>)) == 0;
  }

  Vector3<T> v{Vector3<T>::Constant(nan())};
  T m{nan()};
  Vector3<T> scratch{Vector3<T>::Constant(nan())};
  typename std::conditional<std::is_same_v<T, float>, GridNodeIndex<int32_t>,
                            GridNodeIndex<int64_t>>::type index{};
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
