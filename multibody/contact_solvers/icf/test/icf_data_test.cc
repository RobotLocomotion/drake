#include "drake/multibody/contact_solvers/icf/icf_data.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Checks that a default constructed data object is empty. */
GTEST_TEST(IcfData, DefaultConstructedIsEmpty) {
  IcfData<double> data;

  EXPECT_EQ(data.num_velocities(), 0);
  EXPECT_EQ(data.V_WB().size(), 0);
  EXPECT_EQ(data.scratch().Av_minus_r.size(), 0);
  EXPECT_EQ(data.scratch().V_WB_alpha.size(), 0);
  EXPECT_EQ(data.scratch().v_alpha.size(), 0);
  EXPECT_EQ(data.scratch().Gw_gain.size(), 0);
  EXPECT_EQ(data.scratch().Gw_limit.size(), 0);
  EXPECT_EQ(data.scratch().U_AbB_W.size(), 0);
  EXPECT_EQ(data.scratch().H_BB_pool.size(), 0);
  EXPECT_EQ(data.scratch().H_AA_pool.size(), 0);
  EXPECT_EQ(data.scratch().H_AB_pool.size(), 0);
  EXPECT_EQ(data.scratch().H_BA_pool.size(), 0);
  EXPECT_EQ(data.scratch().GJa_pool.size(), 0);
  EXPECT_EQ(data.scratch().GJb_pool.size(), 0);
}

/* Checks that elements are the correct shape after a resize. */
GTEST_TEST(IcfData, ResizeAndAccessors) {
  IcfData<double> data;
  const int num_bodies = 5;
  const int num_velocities = 12;
  const int max_clique_size = 6;
  const int num_couplers = 2;
  const std::vector<int> gain_sizes = {3, 2};
  const std::vector<int> limit_sizes = {5, 4, 3};
  const std::vector<int> patch_sizes = {8, 6, 4, 2};

  data.Resize(num_bodies, num_velocities, max_clique_size, num_couplers,
              gain_sizes, limit_sizes, patch_sizes);

  // Main data elements
  EXPECT_EQ(data.num_velocities(), num_velocities);
  EXPECT_EQ(data.v().size(), num_velocities);
  EXPECT_EQ(data.V_WB().size(), num_bodies);
  EXPECT_EQ(data.Av().size(), num_velocities);
  EXPECT_EQ(data.gradient().size(), num_velocities);
  // N.B. scalar cost terms (cost, momentum cost) are considered undefined until
  // set explicitly.

  // Scratch space
  EXPECT_EQ(data.scratch().Av_minus_r[0].size(), num_velocities);
  EXPECT_EQ(data.scratch().V_WB_alpha.size(), num_bodies);
  EXPECT_EQ(data.scratch().U_AbB_W.size(), 4);  // num_patches()
  EXPECT_EQ(data.scratch().v_alpha[0].size(), num_velocities);
  EXPECT_EQ(data.scratch().Gw_gain[0].size(), max_clique_size);
  EXPECT_EQ(data.scratch().Gw_limit[0].size(), max_clique_size);
  EXPECT_EQ(data.scratch().coupler_constraints_data.num_constraints(),
            num_couplers);
  EXPECT_EQ(data.scratch().gain_constraints_data.num_constraints(),
            ssize(gain_sizes));
  EXPECT_EQ(data.scratch().limit_constraints_data.num_constraints(),
            ssize(limit_sizes));
  EXPECT_EQ(data.scratch().patch_constraints_data.num_constraints(),
            ssize(patch_sizes));
  EXPECT_EQ(data.scratch().H_cc_pool[0].rows(), max_clique_size);
  EXPECT_EQ(data.scratch().H_cc_pool[0].cols(), max_clique_size);
  EXPECT_EQ(data.scratch().H_BB_pool[0].rows(), max_clique_size);
  EXPECT_EQ(data.scratch().H_BB_pool[0].cols(), max_clique_size);
  EXPECT_EQ(data.scratch().H_AA_pool[0].rows(), max_clique_size);
  EXPECT_EQ(data.scratch().H_AA_pool[0].cols(), max_clique_size);
  EXPECT_EQ(data.scratch().H_AB_pool[0].rows(), max_clique_size);
  EXPECT_EQ(data.scratch().H_AB_pool[0].cols(), max_clique_size);
  EXPECT_EQ(data.scratch().H_BA_pool[0].rows(), max_clique_size);
  EXPECT_EQ(data.scratch().H_BA_pool[0].cols(), max_clique_size);
  EXPECT_EQ(data.scratch().GJa_pool[0].cols(), max_clique_size);
  EXPECT_EQ(data.scratch().GJb_pool[0].cols(), max_clique_size);
}

/* Checks that calling Resize doesn't cost heap allocations when the new size is
no larger than it has been previously. */
GTEST_TEST(IcfData, LimitMallocOnResize) {
  IcfData<double> data;
  const int num_bodies = 3;
  const int num_velocities = 11;
  const int max_clique_size = 7;
  const int num_couplers = 2;
  const std::vector<int> gain_sizes = {3};
  const std::vector<int> limit_sizes = {4};
  const std::vector<int> patch_sizes = {5};

  data.Resize(num_bodies, num_velocities, max_clique_size, num_couplers,
              gain_sizes, limit_sizes, patch_sizes);

  // Clearing pools changes size but shouldn't change capacity.
  EXPECT_EQ(data.scratch().V_WB_alpha.size(), num_bodies);
  data.scratch().Resize(0, 0, 0, 0, gain_sizes, limit_sizes, patch_sizes);
  EXPECT_EQ(data.scratch().V_WB_alpha.size(), 0);

  VectorX<double> v = VectorX<double>::LinSpaced(num_velocities, 1.0, 11.0);
  {
    // Restoring the data to the original size and setting velocities should not
    // cause any new allocations.
    drake::test::LimitMalloc guard;
    data.Resize(num_bodies, num_velocities, max_clique_size, num_couplers,
                gain_sizes, limit_sizes, patch_sizes);
    data.set_v(v);
  }
  EXPECT_EQ(data.scratch().V_WB_alpha.size(), num_bodies);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
