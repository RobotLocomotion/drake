#include "drake/multibody/contact_solvers/icf/patch_constraints_pool.h"

#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
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
GTEST_TEST(PatchConstraintsPool, LimitMallocOnCalcData) {
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

/* Checks that pool.ReduceInto does not incur any heap allocations on a
problem with patch constraints. */
GTEST_TEST(PatchConstraintsPool, LimitMallocOnReduceInto) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddPatchConstraints(&model);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  // Do a not-smaller reduction to allocate memory in the reduced model.
  MakeModelReducible(&model, {});
  model.ReduceInto(&reduced_model, &mapping);

  // Given prior allocation of a big enough model, the constraint pool
  // reduction does not allocate.
  {
    drake::test::LimitMalloc guard;
    model.patch_constraints_pool().ReduceInto(
        mapping, &reduced_model.patch_constraints_pool());
  }
}

/* Verifies that reducing the patch constraint pool produces correct data. */
GTEST_TEST(PatchConstraintsPool, Reduce) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddPatchConstraints(&model);
  model.patch_constraints_pool().set_stiction_tolerance(1.1e-4);

  IcfData<double> data;
  model.ResizeData(&data);
  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model.CalcData(v, &data);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  auto check_reduced = [&](const std::vector<int>& locked_dofs) {
    SCOPED_TRACE(fmt::format("locked_dofs [{}]", fmt::join(locked_dofs, ", ")));
    MakeModelReducible(&model, locked_dofs);
    model.ReduceInto(&reduced_model, &mapping);

    // Check the data transmitted by pool.ReduceInto().
    const auto& full_pool = model.patch_constraints_pool();
    const auto& reduced_pool = reduced_model.patch_constraints_pool();
    EXPECT_EQ(full_pool.stiction_tolerance(),
              reduced_pool.stiction_tolerance());

    int r_k{0};  // Reduced constraints cursor.
    for (int k = 0; k < full_pool.num_constraints(); ++k) {
      SCOPED_TRACE(
          fmt::format("full constraint {} vs. reduced constraint {}", k, r_k));
      const auto& [b, a] = full_pool.bodies()[k];
      const int clique_b = model.params().body_to_clique[b];
      const int clique_a = model.params().body_to_clique[a];
      const bool have_b = mapping.clique_subsequence.participates(clique_b);
      const bool have_a =
          clique_a >= 0 && mapping.clique_subsequence.participates(clique_a);
      if (!(have_a || have_b)) {
        continue;
      }
      EXPECT_EQ(reduced_pool.num_cliques()[r_k], have_a + have_b);
      EXPECT_EQ(reduced_pool.dissipation()[r_k], full_pool.dissipation()[k]);
      EXPECT_EQ(reduced_pool.static_friction()[r_k],
                full_pool.static_friction()[k]);
      EXPECT_EQ(reduced_pool.dynamic_friction()[r_k],
                full_pool.dynamic_friction()[k]);
      const bool is_flipped = have_a && !have_b;
      const auto& [r_b, r_a] = reduced_pool.bodies()[r_k];
      SCOPED_TRACE(fmt::format("flipped? {} have a? {} have b? {}", is_flipped,
                               have_a, have_b));
      if (is_flipped) {
        EXPECT_EQ(b, r_a);
        EXPECT_EQ(a, r_b);
        EXPECT_EQ(reduced_pool.p_AB_W()[r_k], -full_pool.p_AB_W()[k]);
      } else {
        EXPECT_EQ(b, r_b);
        EXPECT_EQ(a, r_a);
        EXPECT_EQ(reduced_pool.p_AB_W()[r_k], full_pool.p_AB_W()[k]);
      }
      const bool Rt_is_smaller = (reduced_pool.Rt()[r_k] < full_pool.Rt()[k]);
      const bool cliques_changed =
          (reduced_pool.num_cliques()[r_k] != full_pool.num_cliques()[k]);
      // TODO(rpoyner-tri): maybe test Rt in more detail.
      EXPECT_EQ(Rt_is_smaller, cliques_changed);

      // This check relies on there being one pair per patch.
      EXPECT_EQ(reduced_pool.patch_pair_index(r_k, 0), r_k);

      EXPECT_EQ(reduced_pool.num_pairs(r_k), full_pool.num_pairs(k));
      for (int p = 0; p < full_pool.num_pairs(k); ++p) {
        const int f_p = full_pool.patch_pair_index(k, p);
        const int r_p = reduced_pool.patch_pair_index(r_k, p);
        EXPECT_EQ(reduced_pool.stiffness()[r_p], full_pool.stiffness()[f_p]);
        EXPECT_EQ(reduced_pool.fe0()[r_p], full_pool.fe0()[f_p]);
        EXPECT_EQ(reduced_pool.fn0()[r_p], full_pool.fn0()[f_p]);
        EXPECT_EQ(reduced_pool.net_friction()[r_p],
                  full_pool.net_friction()[f_p]);
        if (is_flipped) {
          EXPECT_EQ(reduced_pool.normal_W()[r_p], -full_pool.normal_W()[f_p]);
          // In the flipped case, reduced p_BC_W is the same as full p_AC_W.
          EXPECT_EQ(reduced_pool.p_BC_W()[r_p],
                    full_pool.p_AB_W()[k] + full_pool.p_BC_W()[f_p]);
        } else {
          EXPECT_EQ(reduced_pool.normal_W()[r_p], full_pool.normal_W()[f_p]);
          EXPECT_EQ(reduced_pool.p_BC_W()[r_p], full_pool.p_BC_W()[f_p]);
        }
      }
      ++r_k;
    }
    EXPECT_EQ(reduced_pool.num_constraints(), r_k);
  };

  // Reduce by none; essentially, just copy.
  const std::vector<int> none_locked;
  check_reduced(none_locked);

  // Lock some arbitrary dofs.
  const std::vector<int> arbitrary_locked = {0, 17};
  check_reduced(arbitrary_locked);

  // Lock clique 0.
  const std::vector<int> clique0_locked = {0, 1, 2, 3, 4, 5};
  check_reduced(clique0_locked);

  // Lock clique 1.
  const std::vector<int> clique1_locked = {6, 7, 8, 9, 10, 11};
  check_reduced(clique1_locked);

  // Lock clique 2.
  const std::vector<int> clique2_locked = {12, 13, 14, 15, 16, 17};
  check_reduced(clique2_locked);

  // Lock everything.
  std::vector<int> all_locked(model.num_velocities());
  std::iota(all_locked.begin(), all_locked.end(), 0);
  check_reduced(all_locked);
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
