#include "drake/common/atomic_shared_ptr.h"

#include <gtest/gtest.h>

namespace drake {
namespace internal {
namespace {

GTEST_TEST(AtomicSharedPtrTest, CompileCheck) {
  using DUT = atomic_shared_ptr<int>;

  // Compile check the required typedef.
  // The value_type is the unwrapped shared_ptr<T> (without atomics).
  typename DUT::value_type non_atomic;

  // Compile check the required constructors and assignment.
  DUT empty;
  DUT dut(non_atomic);
  dut = non_atomic;
  non_atomic = dut;

  // Compile check all methods (except for the three unimplemented ones).
  const auto seq_cst = std::memory_order_seq_cst;
  dut.is_lock_free();
  dut.store(non_atomic, seq_cst);
  dut.load(seq_cst);
  dut.exchange(non_atomic, seq_cst);
  dut.compare_exchange_strong(non_atomic, non_atomic, seq_cst, seq_cst);
  dut.compare_exchange_weak(non_atomic, non_atomic, seq_cst, seq_cst);
  dut.compare_exchange_strong(non_atomic, non_atomic, seq_cst);
  dut.compare_exchange_weak(non_atomic, non_atomic, seq_cst);

  // Compile check default arguments.
  dut.store(non_atomic);
  dut.load();
  dut.exchange(non_atomic);
  dut.compare_exchange_strong(non_atomic, non_atomic);
  dut.compare_exchange_weak(non_atomic, non_atomic);
}

}  // namespace
}  // namespace internal
}  // namespace drake
