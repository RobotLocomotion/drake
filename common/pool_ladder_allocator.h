#pragma once

// @file
// Allocate arrays of doubles from a sequence of memory pools, graduated in
// size.

#include <algorithm>
#include <atomic>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace internal {
namespace pool_ladder_allocator {

// A pool of arrays that get reused without returning storage to the global
// heap. Use this to speed up computations that would otherwise thrash the heap
// creating and destroying many arrays of similar size.
template <int Power>
class Pool {
 public:
  static constexpr int kSize = 1 << Power;
  static constexpr int kPrePopulation = 5;
  Pool() {
  }

  // Get an object from the pool, making some from the global heap if none are
  // available. Use this where you would otherwise use `new` or malloc().
  inline double* get() {
    if (pool_.empty()) {
      for (int k = 0; k < kPrePopulation; ++k) {
        put(new double[kSize]);
      }
    }
    double* result{};
    result = pool_.back();
    pool_.pop_back();
    return result;
  }

  // Put an object in the pool. Use this where you would otherwise use `delete`
  // or free().
  inline void put(double* p) {
    if (!p) return;
    pool_.push_back(p);
  }

  // Report the number of objects in the pool.
  // The primary use of this method is for testing.
  inline int size() const {
    return static_cast<int>(pool_.size());
  }

  ~Pool() {
    for (auto p : pool_) {
      delete[] p;
    }
  }

 private:
  std::vector<double*> pool_;
};

// Return the log2() of the next larger power of 2.
// @postcondition: x <= 1 << ceil_log2(x)
// @postcondition: x > 1 << ceil_log2(x - 1)
inline int ceil_log2(int x) {
  if (x <= 1) return 1;
  return sizeof(int) * 8 - __builtin_clz(x - 1);
}

// A sequence of pools of power-of-2 graduated sizes.
class Ladder {
 public:
  Ladder() {
    ++s_instance_count_;
  }

  ~Ladder() {
    --s_instance_count_;
  }

  static int instance_count() {
    return s_instance_count_.load();
  }

  inline double* get(int size) {
    int internal_size = size + 1;
    int power = ceil_log2(internal_size);
    double* internal_ptr{};
    switch (power) {
      case 1:
      case 2:
      case 3:
      case 4:
      case 5: internal_ptr = pool5_.get(); break;
      case 6: internal_ptr = pool6_.get(); break;
      case 7: internal_ptr = pool7_.get(); break;
      case 8: internal_ptr = pool8_.get(); break;
      case 9: internal_ptr = pool9_.get(); break;
      case 10: internal_ptr = pool10_.get(); break;
      case 11: internal_ptr = pool11_.get(); break;
      case 12: internal_ptr = pool12_.get(); break;
      case 13: internal_ptr = pool13_.get(); break;
      case 14: internal_ptr = pool14_.get(); break;
      case 15: internal_ptr = pool15_.get(); break;
      default: {
        // Wow! That's a lot of derivatives!!
        DRAKE_DEMAND(power > 0 && power <= 15);
      }
    }
    *internal_ptr = power;
    return internal_ptr + 1;
  }

  inline void put(double* p) {
    double* internal_ptr = p - 1;
    int power = *internal_ptr;
    switch (power) {
      case 1:
      case 2:
      case 3:
      case 4:
      case 5: pool5_.put(internal_ptr); break;
      case 6: pool6_.put(internal_ptr); break;
      case 7: pool7_.put(internal_ptr); break;
      case 8: pool8_.put(internal_ptr); break;
      case 9: pool9_.put(internal_ptr); break;
      case 10: pool10_.put(internal_ptr); break;
      case 11: pool11_.put(internal_ptr); break;
      case 12: pool12_.put(internal_ptr); break;
      case 13: pool13_.put(internal_ptr); break;
      case 14: pool14_.put(internal_ptr); break;
      case 15: pool15_.put(internal_ptr); break;
      default: {
        // Wow! That's a lot of derivatives!!
        DRAKE_DEMAND(power > 0 && power <= 15);
      }
    }
  }

 private:
  Pool<5> pool5_;
  Pool<6> pool6_;
  Pool<7> pool7_;
  Pool<8> pool8_;
  Pool<9> pool9_;
  Pool<10> pool10_;
  Pool<11> pool11_;
  Pool<12> pool12_;
  Pool<13> pool13_;
  Pool<14> pool14_;
  Pool<15> pool15_;

  static std::atomic_int s_instance_count_;
};

}  // namespace pool_ladder_allocator
}  // namespace internal
}  // namespace drake
