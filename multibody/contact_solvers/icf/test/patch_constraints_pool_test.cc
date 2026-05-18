#include "drake/multibody/contact_solvers/icf/patch_constraints_pool.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/test_utilities/icf_model_test_helpers.h"

using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

/* Checks that model.CalcData does not incur any heap allocations for a model
with contact constraints. */
GTEST_TEST(IcfModel, LimitMallocOnPatchConstrainedCalcData) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddPatchConstraints(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 3);
  EXPECT_EQ(model.patch_constraints_pool().num_constraints(), 3);

  IcfData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.patch_constraints_data().num_constraints(), 3);

  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10.0, 10.0);

  // Computing data should not cause any new allocations.
  {
    drake::test::LimitMalloc guard;
    model.CalcData(v, &data);
  }
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
