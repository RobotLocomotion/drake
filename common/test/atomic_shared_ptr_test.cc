#include "drake/common/atomic_shared_ptr.h"

#include <gtest/gtest.h>

namespace drake {
namespace internal {
namespace {

// The typedef implementation (`using atomic_shared_ptr = ...`) doesn't need a
// unit test. For the polyfill implementation, our primary tool for finding bugs
// is code inspection during review, not unit tests. The leverage of a unit test
// to be able to find a memory ordering bug or race condition is very low. We'll
// merely check that the supported API compiles, to find any easy-to-miss typos
// in the signatures.

GTEST_TEST(AtomicSharedPtrTest, CompileCheck) {
  using DUT = atomic_shared_ptr<int>;

  // Compile check the required typedef.
  // The value_type is the unwrapped shared_ptr<T> (without atomics).
  typename DUT::value_type non_atomic;

  // Compile check the required constant.
  [[maybe_unused]] constexpr bool ignore = DUT::is_always_lock_free;

  // Compile check the required constructors and assignment.
  DUT empty;
  DUT explicitly_empty(nullptr);
  DUT dut(non_atomic);
  const DUT& const_dut = dut;
  dut = non_atomic;
  non_atomic = const_dut;

  // Compile check all methods (except for the three unimplemented ones).
  const auto seq_cst = std::memory_order_seq_cst;
  const_dut.is_lock_free();
  dut.store(non_atomic, seq_cst);
  const_dut.load(seq_cst);
  dut.exchange(non_atomic, seq_cst);
  dut.compare_exchange_strong(non_atomic, non_atomic, seq_cst, seq_cst);
  dut.compare_exchange_weak(non_atomic, non_atomic, seq_cst, seq_cst);
  dut.compare_exchange_strong(non_atomic, non_atomic, seq_cst);
  dut.compare_exchange_weak(non_atomic, non_atomic, seq_cst);

  // Compile check default arguments.
  dut.store(non_atomic);
  const_dut.load();
  dut.exchange(non_atomic);
  dut.compare_exchange_strong(non_atomic, non_atomic);
  dut.compare_exchange_weak(non_atomic, non_atomic);
}

}  // namespace
}  // namespace internal
}  // namespace drake
