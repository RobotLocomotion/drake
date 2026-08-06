#include "drake/multibody/contact_solvers/icf/patch_constraints_pool.h"

#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/test_utilities/icf_model_test_helpers.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

/* Re-supplies every pair in `pool` with its existing geometry plus the given
per-pair surface-velocity bias (indexed by patch_pair_index). */
void InjectSurfaceVelocityBias(PatchConstraintsPool<double>* pool,
                               const std::vector<Vector3d>& biases) {
  for (int p = 0; p < pool->num_patches(); ++p) {
    for (int k = 0; k < pool->num_pairs(p); ++k) {
      const int pk = pool->patch_pair_index(p, k);
      pool->SetPair(p, k, pool->p_BC_W()[pk], pool->normal_W()[pk],
                    pool->fe0()[pk], pool->stiffness()[pk], biases.at(pk));
    }
  }
}

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

      // Determine from the mapping if and how the constraint should have been
      // reduced.
      const auto& [b, a] = full_pool.bodies()[k];
      const int clique_b = model.params().body_to_clique[b];
      const int clique_a = model.params().body_to_clique[a];
      const bool have_b = mapping.clique_subsequence.participates(clique_b);
      const bool have_a =
          clique_a >= 0 && mapping.clique_subsequence.participates(clique_a);
      if (!(have_a || have_b)) {
        continue;
      }
      // Gather some facts about clique changes.
      const bool is_flipped = have_a && !have_b;
      const bool cliques_changed =
          (reduced_pool.num_cliques()[r_k] != full_pool.num_cliques()[k]);
      SCOPED_TRACE(fmt::format("flipped? {} have a? {} have b? {}", is_flipped,
                               have_a, have_b));

      EXPECT_EQ(reduced_pool.num_pairs(r_k), full_pool.num_pairs(k));
      EXPECT_EQ(reduced_pool.num_cliques()[r_k], have_a + have_b);
      // TODO(rpoyner-tri): maybe test Rt in more detail.
      const bool Rt_is_smaller = (reduced_pool.Rt()[r_k] < full_pool.Rt()[k]);
      EXPECT_EQ(Rt_is_smaller, cliques_changed);
      // Test quantities sensitive to flipping.
      const auto& [r_b, r_a] = reduced_pool.bodies()[r_k];
      if (is_flipped) {
        EXPECT_EQ(b, r_a);
        EXPECT_EQ(a, r_b);
        EXPECT_EQ(reduced_pool.p_AB_W()[r_k], -full_pool.p_AB_W()[k]);
      } else {
        EXPECT_EQ(b, r_b);
        EXPECT_EQ(a, r_a);
        EXPECT_EQ(reduced_pool.p_AB_W()[r_k], full_pool.p_AB_W()[k]);
      }
      EXPECT_EQ(reduced_pool.dissipation()[r_k], full_pool.dissipation()[k]);
      EXPECT_EQ(reduced_pool.static_friction()[r_k],
                full_pool.static_friction()[k]);
      EXPECT_EQ(reduced_pool.dynamic_friction()[r_k],
                full_pool.dynamic_friction()[k]);

      // This check relies on there being one pair per patch.
      EXPECT_EQ(reduced_pool.patch_pair_index(r_k, 0), r_k);

      for (int q = 0; q < full_pool.num_pairs(k); ++q) {
        const int p = full_pool.patch_pair_index(k, q);
        const int r_p = reduced_pool.patch_pair_index(r_k, q);
        // Test quantities sensitive to flipping.
        if (is_flipped) {
          // In the flipped case, reduced p_BC_W is the same as full p_AC_W.
          EXPECT_EQ(reduced_pool.p_BC_W()[r_p],
                    full_pool.p_AB_W()[k] + full_pool.p_BC_W()[p]);
          EXPECT_EQ(reduced_pool.normal_W()[r_p], -full_pool.normal_W()[p]);
        } else {
          EXPECT_EQ(reduced_pool.p_BC_W()[r_p], full_pool.p_BC_W()[p]);
          EXPECT_EQ(reduced_pool.normal_W()[r_p], full_pool.normal_W()[p]);
        }
        EXPECT_EQ(reduced_pool.stiffness()[r_p], full_pool.stiffness()[p]);
        EXPECT_EQ(reduced_pool.fe0()[r_p], full_pool.fe0()[p]);
        EXPECT_EQ(reduced_pool.fn0()[r_p], full_pool.fn0()[p]);
        EXPECT_EQ(reduced_pool.net_friction()[r_p],
                  full_pool.net_friction()[p]);
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

/* The live per-pair contact velocity computed by CalcData() must be the
relative velocity J⋅v plus the surface-velocity bias v_b. We verify this across
all three patch topologies exercised by AddPatchConstraints() (floating body A,
floating body B, and an anchored body A). */
GTEST_TEST(PatchConstraintsPool, CalcDataWithSurfaceVelocityBias) {
  // Reference model, with zero bias.
  IcfModel<double> model_ref;
  MakeUnconstrainedModel(&model_ref);
  AddPatchConstraints(&model_ref);

  // Biased model: identical, but with a distinct bias injected per pair.
  IcfModel<double> model_bias;
  MakeUnconstrainedModel(&model_bias);
  AddPatchConstraints(&model_bias);
  const std::vector<Vector3d> biases = {Vector3d(0.1, 0.2, 0.3),
                                        Vector3d(-0.4, 0.5, -0.6),
                                        Vector3d(0.7, -0.8, 0.9)};
  InjectSurfaceVelocityBias(&model_bias.patch_constraints_pool(), biases);

  const int nv = model_ref.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10.0, 10.0);

  IcfData<double> data_ref;
  model_ref.ResizeData(&data_ref);
  model_ref.CalcData(v, &data_ref);

  IcfData<double> data_bias;
  model_bias.ResizeData(&data_bias);
  model_bias.CalcData(v, &data_bias);

  const EigenPool<Vector3<double>>& v_ref =
      data_ref.patch_constraints_data().v_AcBc_W_pool();
  const EigenPool<Vector3<double>>& v_bias =
      data_bias.patch_constraints_data().v_AcBc_W_pool();
  const PatchConstraintsPool<double>& pool =
      model_bias.patch_constraints_pool();
  ASSERT_EQ(pool.total_num_pairs(), ssize(biases));
  for (int pk = 0; pk < pool.total_num_pairs(); ++pk) {
    SCOPED_TRACE(fmt::format("pair {}", pk));
    EXPECT_TRUE(CompareMatrices(v_bias[pk], v_ref[pk] + biases[pk], 1e-13));
  }
}

/* The lagged, previous-step normal force fn0 is computed in SetPair() from the
previous-step contact velocity, which must include the surface-velocity bias.
We verify this by contrasting a strongly separating vs. strongly approaching
normal bias on the same pair: a separating bias over-damps the Hunt & Crossley
model (fn0 clamped to zero), while an approaching bias yields fn0 > 0. This is
robust to the (unknown) baseline contact velocity of the test fixture. */
GTEST_TEST(PatchConstraintsPool, LaggedQuantitiesWithSurfaceVelocityBias) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddPatchConstraints(&model);
  auto& pool = model.patch_constraints_pool();

  // Patch 1 is the anchored (world) + dynamic-body patch, with normal +ŷ. Its
  // fe0 = 1.5 > 0, so fn0 is driven purely by the Hunt & Crossley damping term.
  const int pk = pool.patch_pair_index(1, 0);
  std::vector<Vector3d> biases(pool.total_num_pairs(), Vector3d::Zero());

  // Strongly separating (+ŷ) bias: bodies move apart, damping → 0, fn0 = 0.
  biases[pk] = Vector3d(0.0, 100.0, 0.0);
  InjectSurfaceVelocityBias(&pool, biases);
  const double fn0_separating = pool.fn0()[pk];

  // Strongly approaching (−ŷ) bias: the damping term stays positive, fn0 > 0.
  biases[pk] = Vector3d(0.0, -100.0, 0.0);
  InjectSurfaceVelocityBias(&pool, biases);
  const double fn0_approaching = pool.fn0()[pk];

  EXPECT_EQ(fn0_separating, 0.0);
  EXPECT_GT(fn0_approaching, 0.0);
}

/* ReduceInto() must carry the per-pair surface-velocity bias through to the
reduced pool, negating it whenever the body pair is flipped (since
v_b = v_B_ss - v_A_ss changes sign when A and B are swapped). */
GTEST_TEST(PatchConstraintsPool, ReduceWithSurfaceVelocity) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddPatchConstraints(&model);
  const std::vector<Vector3d> biases = {Vector3d(0.1, 0.2, 0.3),
                                        Vector3d(-0.4, 0.5, -0.6),
                                        Vector3d(0.7, -0.8, 0.9)};
  InjectSurfaceVelocityBias(&model.patch_constraints_pool(), biases);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  auto check_reduced = [&](const std::vector<int>& locked_dofs) {
    SCOPED_TRACE(fmt::format("locked_dofs [{}]", fmt::join(locked_dofs, ", ")));
    MakeModelReducible(&model, locked_dofs);
    model.ReduceInto(&reduced_model, &mapping);

    const auto& full_pool = model.patch_constraints_pool();
    const auto& reduced_pool = reduced_model.patch_constraints_pool();

    int r_k{0};
    for (int k = 0; k < full_pool.num_constraints(); ++k) {
      const auto& [b, a] = full_pool.bodies()[k];
      const int clique_b = model.params().body_to_clique[b];
      const int clique_a = model.params().body_to_clique[a];
      const bool have_b = mapping.clique_subsequence.participates(clique_b);
      const bool have_a =
          clique_a >= 0 && mapping.clique_subsequence.participates(clique_a);
      if (!(have_a || have_b)) continue;
      const bool is_flipped = have_a && !have_b;
      for (int q = 0; q < full_pool.num_pairs(k); ++q) {
        const int p = full_pool.patch_pair_index(k, q);
        const int r_p = reduced_pool.patch_pair_index(r_k, q);
        const Vector3d expected =
            is_flipped ? Vector3d(-full_pool.v_b_W()[p]) : full_pool.v_b_W()[p];
        EXPECT_TRUE(CompareMatrices(reduced_pool.v_b_W()[r_p], expected, 0.0));
      }
      ++r_k;
    }
  };

  check_reduced({});                  // Copy (no reduction).
  check_reduced({0, 1, 2, 3, 4, 5});  // Lock clique 0.
  // Lock clique 1; body 1 becomes anchored, flipping patch 0 (A=2, B=1).
  check_reduced({6, 7, 8, 9, 10, 11});
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
