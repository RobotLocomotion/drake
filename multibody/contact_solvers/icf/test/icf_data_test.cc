#include "drake/multibody/contact_solvers/icf/icf_data.h"

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

  data.Resize(num_bodies, num_velocities, max_clique_size);

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
  EXPECT_EQ(data.scratch().v_alpha[0].size(), num_velocities);
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

/* Checks that calling Resize doesn't cost extra heap allocations. */
GTEST_TEST(IcfData, LimitMallocOnResize) {
  IcfData<double> data;
  const int num_bodies = 3;
  const int num_velocities = 11;
  const int max_clique_size = 7;

  data.Resize(num_bodies, num_velocities, max_clique_size);

  // Clearing pools changes size but shouldn't change capacity.
  EXPECT_EQ(data.scratch().V_WB_alpha.size(), num_bodies);
  data.scratch().Clear();
  EXPECT_EQ(data.scratch().V_WB_alpha.size(), 0);

  VectorX<double> v = VectorX<double>::LinSpaced(num_velocities, 1.0, 11.0);
  {
    // Restoring the data to the original size and setting velocities should not
    // cause any new allocations.
    drake::test::LimitMalloc guard;
    data.Resize(num_bodies, num_velocities, max_clique_size);
    data.set_v(v);
  }
  EXPECT_EQ(data.scratch().V_WB_alpha.size(), num_bodies);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
