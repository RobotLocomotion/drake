#include "drake/common/test/hwy_dynamic_test_array_mul.h"

#include <fmt/format.h>

// This is the magic juju that compiles our impl functions for multiple CPUs.
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "common/test/hwy_dynamic_test_array_mul.cc"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

#include "drake/common/drake_assert.h"
#include "drake/common/hwy_dynamic_impl.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace {
// In the GetMutableArrayMulCounters() dictionary available in our header file,
// increments the counter with the given name. This is just the declaration;
// the definition appears later in this file.
void IncrementCounter(const std::string& name);
}  // namespace

HWY_BEFORE_NAMESPACE();
namespace drake {
namespace internal {
namespace {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;

void ArrayMulImpl(const Eigen::ArrayXd& x0, const Eigen::ArrayXd& x1,
                  Eigen::ArrayXd* y) {
  DRAKE_DEMAND(x0.size() == x1.size());
  DRAKE_DEMAND(y != nullptr);

  const int size = x0.size();
  y->resize(size);
  const double* const x0_data = x0.data();
  const double* const x1_data = x1.data();
  double* const y_data = y->data();

  const hn::ScalableTag<double> d;
  const int N = hn::Lanes(d);
  int i = 0;
  for (; i + N <= size; i += N) {
    auto x0_vec = hn::LoadU(d, x0_data + i);
    auto x1_vec = hn::LoadU(d, x1_data + i);
    auto y_vec = hn::Mul(x0_vec, x1_vec);
    hn::StoreU(y_vec, d, y_data + i);
  }
  for (; i < size; ++i) {
    y_data[i] = x0_data[i] * x1_data[i];
  }

  IncrementCounter(fmt::format("ArrayMulImpl.{}", hwy::TargetName(HWY_TARGET)));
}

}  // namespace HWY_NAMESPACE
}  // namespace
}  // namespace internal
}  // namespace drake
HWY_AFTER_NAMESPACE();

// This part of the file is only compiled once total, instead of once per CPU.
#if HWY_ONCE
namespace {
void IncrementCounter(const std::string& name) {
  drake::log()->info("++{}", name);
  ++(drake::internal::GetMutableArrayMulCounters()[name]);
}
}  // namespace
namespace drake {
namespace internal {
namespace {

// Create the lookup tables for the per-CPU hwy implementation functions, and
// required functors that select from the lookup tables.
HWY_EXPORT(ArrayMulImpl);
struct ChooseBestArrayMul {
  auto operator()() {
    IncrementCounter("ChooseBestArrayMul");
    return HWY_DYNAMIC_POINTER(ArrayMulImpl);
  }
};

}  // namespace

void ArrayMul(const Eigen::ArrayXd& x0, const Eigen::ArrayXd& x1,
              Eigen::ArrayXd* y1) {
  LateBoundFunction<ChooseBestArrayMul>::Call(x0, x1, y1);
}

string_map<int>& GetMutableArrayMulCounters() {
  static drake::never_destroyed<string_map<int>> singleton;
  return singleton.access();
}

}  // namespace internal
}  // namespace drake
#endif  // HWY_ONCE
