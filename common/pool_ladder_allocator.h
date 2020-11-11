#pragma once

// @file
// Allocate arrays of doubles from a sequence of memory pools, graduated in
// size. This allocator is not thread-safe; ; the design leaves synchronization
// to higher software layers. Handling of various corner cases is also left to
// higher layers, to optimize and simplify the fast path.

#include <atomic>
#include <limits>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace internal {
namespace pool_ladder_allocator {

// A pool of arrays that get reused without returning storage to the global
// heap. Use this to speed up computations that would otherwise thrash the heap
// creating and destroying many arrays of similar size. Pool is not
// thread-safe.
template <int Power>
class Pool {
 public:
  static constexpr int kSize = 1 << Power;
  static constexpr int kPrePopulation = 5;
  Pool() {
  }

  // Get an object from the pool, making some from the global heap if none are
  // available.
  inline double* Get() {
    if (pool_.empty()) {
      for (int k = 0; k < kPrePopulation; ++k) {
        Put(new double[kSize]);
      }
    }
    double* result = pool_.back();
    pool_.pop_back();
    return result;
  }

  // Put an object in the pool.
  // @precondition: p != nullptr
  inline void Put(double* p) {
    DRAKE_ASSERT(p);
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
// @precondition: x > 1
// @postcondition: x <= (1 << ceil_log2(x))
// @postcondition: x > (1 << ceil_log2(x - 1))
inline int ceil_log2(unsigned int x) {
  DRAKE_ASSERT(x > 1);
  return std::numeric_limits<unsigned int>::digits - __builtin_clz(x - 1);
}

// A sequence of pools of power-of-2 graduated sizes. Ladder is not
// thread-safe, with the exception of the instance count provided for testing.
class Ladder {
 public:
  // These limits could be extended to support larger objects.
  static constexpr int kMaxPower = 15;
  static constexpr int kOverhead = 1;  // Storage used by the allocator.
  static constexpr int kMaxSize = (1 << kMaxPower) - kOverhead;

  Ladder() {
    ++s_instance_count_;
  }

  ~Ladder() {
    --s_instance_count_;
  }

  // Report the number of Ladder instances in existence globally.
  // The primary use of this method is for testing.
  static int instance_count() {
    return s_instance_count_.load();
  }

  // Get an object that can contain at least `size` doubles.
  // @precondition: size > 0
  // @precondition: size <= kMaxSize
  inline double* Get(int size) {
    DRAKE_ASSERT(size > 0);
    const int internal_size = size + kOverhead;
    const int power = ceil_log2(internal_size);
    double* internal_ptr{};
    switch (power) {
      case 1:
      case 2:
      case 3:
      case 4:
      case 5: internal_ptr = pool5_.Get(); break;
      case 6: internal_ptr = pool6_.Get(); break;
      case 7: internal_ptr = pool7_.Get(); break;
      case 8: internal_ptr = pool8_.Get(); break;
      case 9: internal_ptr = pool9_.Get(); break;
      case 10: internal_ptr = pool10_.Get(); break;
      case 11: internal_ptr = pool11_.Get(); break;
      case 12: internal_ptr = pool12_.Get(); break;
      case 13: internal_ptr = pool13_.Get(); break;
      case 14: internal_ptr = pool14_.Get(); break;
      case 15: internal_ptr = pool15_.Get(); break;
      default: {
        // If you really need large objects, consider extending this class.
        DRAKE_DEMAND(power > 0 && power <= 15);
      }
    }
    // Overhead space is used to remember the source pool.
    static_assert(sizeof(int) <= sizeof(double), "need more overhead space");
    *reinterpret_cast<int*>(internal_ptr) = power;
    return internal_ptr + kOverhead;
  }

  // Put an object acquired with Get() back in the appropriate pool.
  // @precondition: p != nullptr
  // @precondition: p = Get(x) -- only partially checked
  inline void Put(double* p) {
    DRAKE_ASSERT(p);
    double* internal_ptr = p - kOverhead;
    // Recover pool information from overhead space.
    const int power = *reinterpret_cast<int*>(internal_ptr);
    switch (power) {
      case 1:
      case 2:
      case 3:
      case 4:
      case 5: pool5_.Put(internal_ptr); break;
      case 6: pool6_.Put(internal_ptr); break;
      case 7: pool7_.Put(internal_ptr); break;
      case 8: pool8_.Put(internal_ptr); break;
      case 9: pool9_.Put(internal_ptr); break;
      case 10: pool10_.Put(internal_ptr); break;
      case 11: pool11_.Put(internal_ptr); break;
      case 12: pool12_.Put(internal_ptr); break;
      case 13: pool13_.Put(internal_ptr); break;
      case 14: pool14_.Put(internal_ptr); break;
      case 15: pool15_.Put(internal_ptr); break;
      default: {
        // The input block was not made by Get(), or other code violated array
        // bounds by overwriting p[-1].
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
