#include "drake/common/test_utilities/diagnostic_policy_test_base.h"

namespace drake {
namespace test {

DiagnosticPolicyTestBase::~DiagnosticPolicyTestBase() {
  FlushDiagnostics();
}

}  // namespace test
}  // namespace drake
