#include "drake/systems/n_ary_state.h"

#include <stdexcept>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/core/Vector.h"

namespace {

using Drake::NullVector;
using Drake::toEigen;
using drake::NAryState;

// Vector-concept class for exercising composition.
template <typename ScalarType>
struct VectorQ {
  static const int RowsAtCompileTime = Eigen::Dynamic;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  VectorQ() {}

  template <typename Derived>
  explicit VectorQ(const Eigen::MatrixBase<Derived>& initial)
      : v(initial) {}

  template <typename Derived>
  VectorQ& operator=(const Eigen::MatrixBase<Derived>& rhs) {
    v = rhs;
  }

  friend EigenType toEigen(const VectorQ<ScalarType>& vq) {
    return vq.v;
  }

  std::size_t size() const { return 3; }

  EigenType v;
};

using VectorQD = VectorQ<double>;


GTEST_TEST(TestNAryState, UnitCountFromRows) {
  NAryState<NullVector<double> >::UnitCountFromRows(0);

  EXPECT_EQ((NAryState<VectorQD>::UnitCountFromRows(0)), 0u);
  EXPECT_EQ((NAryState<VectorQD>::UnitCountFromRows(15)), 5u);
  EXPECT_THROW((NAryState<VectorQD>::UnitCountFromRows(17)),
               std::domain_error);

  EXPECT_EQ((NAryState<NullVector<double> >::UnitCountFromRows(0)), -1);
  EXPECT_EQ((NAryState<NullVector<double> >::UnitCountFromRows(15)), -1);
}


GTEST_TEST(TestNAryState, RowsFromUnitCount) {
  EXPECT_EQ((NAryState<VectorQD>::RowsFromUnitCount(0)),  0u);
  EXPECT_EQ((NAryState<VectorQD>::RowsFromUnitCount(1)),  3u);
  EXPECT_EQ((NAryState<VectorQD>::RowsFromUnitCount(5)), 15u);
  EXPECT_THROW((NAryState<VectorQD>::RowsFromUnitCount(-1)),
               std::domain_error);

  EXPECT_EQ((NAryState<NullVector<double> >::RowsFromUnitCount(1)), 0u);
  EXPECT_EQ((NAryState<NullVector<double> >::RowsFromUnitCount(5)), 0u);
  EXPECT_EQ((NAryState<NullVector<double> >::RowsFromUnitCount(-1)), 0u);
}


GTEST_TEST(TestNAryState, DefaultConstructor) {
  VectorQD vq0 = VectorQD((Eigen::VectorXd(3) << 1.0, 2.0, 3.0).finished());
  VectorQD vq1 = VectorQD((Eigen::VectorXd(3) << 4.0, 6.0, 8.0).finished());

  // Test default-constructed instance.
  NAryState<VectorQD> dut;
  EXPECT_EQ(dut.count(), 0u);
  EXPECT_EQ(dut.size(), 0u);
  EXPECT_THROW(dut.get(0), std::out_of_range);
  EXPECT_THROW(dut.set(0, VectorQD()), std::out_of_range);
  EXPECT_EQ(toEigen(dut), (Eigen::Matrix<double, 0, 1>()));

  dut.Append(vq0);
  EXPECT_EQ(dut.count(), 1u);
  EXPECT_EQ(dut.size(), 3u);
  EXPECT_EQ(dut.get(0).v, vq0.v);
  EXPECT_EQ(toEigen(dut), vq0.v);

  dut.Append(vq1);
  EXPECT_EQ(dut.count(), 2u);
  EXPECT_EQ(dut.size(), 6u);
  EXPECT_EQ(dut.get(0).v, vq0.v);
  EXPECT_EQ(dut.get(1).v, vq1.v);
  EXPECT_EQ(toEigen(dut),
            (Eigen::VectorXd(6) << vq0.v, vq1.v).finished());

  EXPECT_THROW(dut.get(2), std::out_of_range);
  EXPECT_THROW(dut.set(2, vq1), std::out_of_range);

  dut.set(0, vq1);
  dut.set(1, vq0);
  EXPECT_EQ(dut.get(0).v, vq1.v);
  EXPECT_EQ(dut.get(1).v, vq0.v);
  EXPECT_EQ(toEigen(dut),
            (Eigen::VectorXd(6) << vq1.v, vq0.v).finished());
}


GTEST_TEST(TestNAryState, PreAllocated) {
  VectorQD vq0 = VectorQD((Eigen::VectorXd(3) << 1.0, 2.0, 3.0).finished());
  VectorQD vq1 = VectorQD((Eigen::VectorXd(3) << 4.0, 6.0, 8.0).finished());

  // Test construction with pre-allocated size.
  NAryState<VectorQD> dut(2);
  EXPECT_EQ(dut.count(), 2u);
  EXPECT_EQ(dut.size(), 6u);

  dut.set(0, vq0);
  dut.set(1, vq1);
  EXPECT_EQ(dut.get(0).v, vq0.v);
  EXPECT_EQ(dut.get(1).v, vq1.v);
  EXPECT_EQ(toEigen(dut),
            (Eigen::VectorXd(6) << vq0.v, vq1.v).finished());

  EXPECT_THROW(dut.get(2), std::out_of_range);
  EXPECT_THROW(dut.set(2, vq1), std::out_of_range);
}


GTEST_TEST(TestNAryState, FromEigen) {
  VectorQD vq0 = VectorQD((Eigen::VectorXd(3) << 1.0, 2.0, 3.0).finished());
  VectorQD vq1 = VectorQD((Eigen::VectorXd(3) << 4.0, 6.0, 8.0).finished());

  // Test construction from Eigen type.
  NAryState<VectorQD> dut((
      Eigen::VectorXd(6) << vq0.v, vq1.v).finished());

  EXPECT_EQ(dut.count(), 2u);
  EXPECT_EQ(dut.size(), 6u);
  EXPECT_EQ(dut.get(0).v, vq0.v);
  EXPECT_EQ(dut.get(1).v, vq1.v);
  EXPECT_EQ(toEigen(dut),
            (Eigen::VectorXd(6) << vq0.v, vq1.v).finished());

  EXPECT_THROW(dut.get(2), std::out_of_range);
  EXPECT_THROW(dut.set(2, vq1), std::out_of_range);

  // Test assignment from Eigen type.
  dut = vq0.v;
  EXPECT_EQ(dut.count(), 1u);
  EXPECT_EQ(dut.size(), 3u);
  EXPECT_EQ(dut.get(0).v, vq0.v);
  EXPECT_EQ(toEigen(dut), vq0.v);

  dut = Eigen::VectorXd(0);
  EXPECT_EQ(dut.count(), 0u);
  EXPECT_EQ(dut.size(), 0u);
  EXPECT_THROW(dut.get(0), std::out_of_range);
  EXPECT_EQ(toEigen(dut), Eigen::VectorXd(0));
}


GTEST_TEST(TestNAryState, NullUnitVectors) {
  NullVector<double> nv;

  // Test default-constructed instance.
  NAryState<NullVector<double> > dut;
  EXPECT_EQ(dut.count(), -1);
  EXPECT_EQ(dut.size(), 0u);
  EXPECT_EQ(dut.get(0), nv);
  EXPECT_EQ(dut.get(1000), nv);
  dut.set(0, nv);
  dut.set(1000, nv);
  EXPECT_EQ(toEigen(dut), Eigen::VectorXd(0));

  // Appending nothing to nothing should not change nothing.
  dut.Append(nv);
  EXPECT_EQ(dut.count(), -1);
  EXPECT_EQ(dut.size(), 0u);
  EXPECT_EQ(dut.get(0), nv);
  EXPECT_EQ(dut.get(1000), nv);
  dut.set(0, nv);
  dut.set(1000, nv);
  EXPECT_EQ(toEigen(dut), Eigen::VectorXd(0));

  // Test assignment from Eigen type.
  dut = Eigen::Matrix<double, 0, 1>();
  EXPECT_EQ(dut.count(), -1);
  EXPECT_EQ(dut.size(), 0u);
  EXPECT_EQ(dut.get(0), nv);
  EXPECT_EQ(dut.get(1000), nv);
  dut.set(0, nv);
  dut.set(1000, nv);
  EXPECT_EQ(toEigen(dut), Eigen::VectorXd(0));

  // Test construction from Eigen type.
  NAryState<NullVector<double> > dut2(Eigen::VectorXd(0));
  EXPECT_EQ(dut2.count(), -1);
  EXPECT_EQ(dut2.size(), 0u);
  EXPECT_EQ(dut2.get(0), nv);
  EXPECT_EQ(dut2.get(1000), nv);
  dut2.set(0, nv);
  dut2.set(1000, nv);
  EXPECT_EQ(toEigen(dut2), Eigen::VectorXd(0));

  // Test construction with pre-allocated size.
  NAryState<NullVector<double> > dut3(7);
  EXPECT_EQ(dut3.count(), -1);
  EXPECT_EQ(dut3.size(), 0u);
  EXPECT_EQ(dut3.get(0), nv);
  EXPECT_EQ(dut3.get(1000), nv);
  dut3.set(0, nv);
  dut3.set(1000, nv);
  EXPECT_EQ(toEigen(dut3), Eigen::VectorXd(0));

  NAryState<NullVector<double> > dut4(-1);
  EXPECT_EQ(dut4.count(), -1);
  EXPECT_EQ(dut4.size(), 0u);
  EXPECT_EQ(dut4.get(0), nv);
  EXPECT_EQ(dut4.get(1000), nv);
  dut4.set(0, nv);
  dut4.set(1000, nv);
  EXPECT_EQ(toEigen(dut4), Eigen::VectorXd(0));
}

}  // namespace
