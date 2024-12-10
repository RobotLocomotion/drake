#pragma once

#include <cstdint>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* A class representing

    1. an index of a grid node, or
    2. a flag for a grid node in preparation of indexing the grid.

  This class is a lightweight wrapper around an integer type (`int32_t` or
  `int64_t`) used to differentiate between active indices and a special flag.
  A GridNodeIndex can be in exactly one of the two states:

    1. Valid index: A non-negative integer representing the index of a grid.
    2. The flag state: A special state used to mark grid nodes for deferred
       processing.

  A GridNodeIndex object is guaranteed to have size equal to its template
  parameter T.

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

  bool operator==(const GridNodeIndex<T>& other) const = default;

  /* Sets the index to the given value, which must be non-negative, thereby
   making `this` a valid index.
   @pre index >= 0 */
  void set_value(T index) {
    DRAKE_ASSERT(index >= 0);
    value_ = index;
  }

  /* Returns true if the index is active (i.e., a non-negative integer). */
  constexpr bool is_valid() const { return value_ >= 0; }

  /* Returns the index value.
   @pre is_valid() == true; */
  constexpr T value() const {
    DRAKE_ASSERT(is_valid());
    return value_;
  }

  /* Sets `this` to be zero.. */
  void reset() { value_ = 0; }

  /* Sets `this` to the flag state. */
  void set_flag() { value_ = kParticipating; }

 private:
  enum : T { kParticipating = -1 };
  T value_{0};
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

  /* Resets `this` GridData to its default state. */
  void reset() {
    v.setZero();
    scratch.setZero();
    m = 0.0;
    index.reset();
  }

  /* Default equality operator to compare all members. */
  bool operator==(const GridData<T>& other) const = default;

  Vector3<T> v{Vector3<T>::Zero()};
  T m{0.0};
  Vector3<T> scratch{Vector3<T>::Zero()};
  typename std::conditional<std::is_same_v<T, float>, GridNodeIndex<int32_t>,
                            GridNodeIndex<int64_t>>::type index;
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
