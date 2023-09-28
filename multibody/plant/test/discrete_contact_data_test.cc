#include "drake/multibody/plant/discrete_contact_data.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

struct DummyData {
  int value;
};

GTEST_TEST(DiscreteContactData, Constructor) {
  DiscreteContactData<DummyData> dut;
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.num_point_contacts(), 0);
  EXPECT_EQ(dut.num_hydro_contacts(), 0);
  EXPECT_EQ(dut.num_deformable_contacts(), 0);
  EXPECT_EQ(dut.point_contact_data().size(), 0);
  EXPECT_EQ(dut.hydro_contact_data().size(), 0);
  EXPECT_EQ(dut.deformable_contact_data().size(), 0);
  EXPECT_THROW(dut[0], std::exception);
}

GTEST_TEST(DiscreteContactData, Size) {
  DiscreteContactData<DummyData> dut;

  dut.AppendPointData(DummyData{100});

  dut.AppendHydroData(DummyData{0});
  dut.AppendHydroData(DummyData{1});

  dut.AppendDeformableData(DummyData{1});
  dut.AppendDeformableData(DummyData{2});
  dut.AppendDeformableData(DummyData{3});

  EXPECT_EQ(dut.num_point_contacts(), 1);
  EXPECT_EQ(dut.num_hydro_contacts(), 2);
  EXPECT_EQ(dut.num_deformable_contacts(), 3);
  EXPECT_EQ(dut.size(), 6);
}

GTEST_TEST(DiscreteContactData, AppendAndAccessAndClear) {
  DiscreteContactData<DummyData> dut;

  dut.AppendPointData(DummyData{1});
  dut.AppendHydroData(DummyData{2});
  dut.AppendDeformableData(DummyData{3});

  /* Access via operator[]. */
  EXPECT_EQ(dut[0].value, 1);
  EXPECT_EQ(dut[1].value, 2);
  EXPECT_EQ(dut[2].value, 3);

  /* Access by group. */
  ASSERT_EQ(dut.point_contact_data().size(), 1);
  EXPECT_EQ(dut.point_contact_data()[0].value, 1);

  ASSERT_EQ(dut.hydro_contact_data().size(), 1);
  EXPECT_EQ(dut.hydro_contact_data()[0].value, 2);

  ASSERT_EQ(dut.deformable_contact_data().size(), 1);
  EXPECT_EQ(dut.deformable_contact_data()[0].value, 3);

  /* Clear */
  dut.Clear();
  EXPECT_EQ(dut.num_point_contacts(), 0);
  EXPECT_EQ(dut.num_hydro_contacts(), 0);
  EXPECT_EQ(dut.num_deformable_contacts(), 0);
  EXPECT_EQ(dut.size(), 0);
}

GTEST_TEST(DiscreteContactData, Reserve) {
  using drake::test::LimitMalloc;
  DiscreteContactData<DummyData> dut;
  dut.Reserve(1, 2, 3);
  /* We should be able to add 1, 2, and 3 point, hydroelastic, and deformable
   data respectively without allocation. */
  LimitMalloc guard;
  dut.AppendPointData(DummyData{0});
  dut.AppendHydroData(DummyData{0});
  dut.AppendHydroData(DummyData{0});
  dut.AppendDeformableData(DummyData{0});
  dut.AppendDeformableData(DummyData{0});
  dut.AppendDeformableData(DummyData{0});
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
