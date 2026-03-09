#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"

// @file
// These bindings help test the public interfaces in `sorted_pair_pybind.h`.
// See also sorted_pair_test.py for the main test code.

#include <map>
#include <string>

namespace drake {
namespace pydrake {

namespace {

SortedPair<std::string> PassThrough(const SortedPair<std::string>& in) {
  return in;
}

using ExampleMap = std::map<SortedPair<std::string>, int>;
ExampleMap PassThroughMap(const ExampleMap& in) {
  return in;
}

}  // namespace

PYBIND11_MODULE(sorted_pair_test_util, m) {
  m.def("PassThrough", &PassThrough);
  m.def("PassThroughMap", &PassThroughMap);
}

}  // namespace pydrake
}  // namespace drake
