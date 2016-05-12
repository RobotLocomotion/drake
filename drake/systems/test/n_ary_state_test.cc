#include "drake/systems/n_ary_state.h"

#include <stdexcept>

#include <Eigen/Dense>

#include "drake/core/Vector.h"

#include "gtest/gtest.h"

namespace {

using Drake::NullVector;
using Drake::toEigen;
using drake::NAryState;

// Vector-concept class for exercising composition.
template <typename ScalarType>
class VectorQ {
 public:
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


GTEST_TEST(TestNAryState, unitCountFromRows) {
  EXPECT_EQ((NAryState<double, VectorQ>::unitCountFromRows( 0)), 0);
  EXPECT_EQ((NAryState<double, VectorQ>::unitCountFromRows(15)), 5);
  EXPECT_THROW((NAryState<double, VectorQ>::unitCountFromRows(17)),
               std::domain_error);

  EXPECT_EQ((NAryState<double, NullVector>::unitCountFromRows( 0)), -1);
  EXPECT_EQ((NAryState<double, NullVector>::unitCountFromRows(15)), -1);
}


GTEST_TEST(TestNAryState, rowsFromUnitCount) {
  EXPECT_EQ((NAryState<double, VectorQ>::rowsFromUnitCount( 0)),  0);
  EXPECT_EQ((NAryState<double, VectorQ>::rowsFromUnitCount( 1)),  3);
  EXPECT_EQ((NAryState<double, VectorQ>::rowsFromUnitCount( 5)), 15);
  EXPECT_THROW((NAryState<double, VectorQ>::rowsFromUnitCount(-1)),
               std::domain_error);

  EXPECT_EQ((NAryState<double, NullVector>::rowsFromUnitCount( 1)), 0);
  EXPECT_EQ((NAryState<double, NullVector>::rowsFromUnitCount( 5)), 0);
  EXPECT_EQ((NAryState<double, NullVector>::rowsFromUnitCount(-1)), 0);
}


GTEST_TEST(TestNAryState, DefaultConstructor) {
  VectorQD vq0 = VectorQD((Eigen::VectorXd(3) << 1.0, 2.0, 3.0).finished());
  VectorQD vq1 = VectorQD((Eigen::VectorXd(3) << 4.0, 6.0, 8.0).finished());

  // Test default-constructed instance.
  NAryState<double, VectorQ> dut;
  EXPECT_EQ(dut.count(), 0);
  EXPECT_EQ(dut.size(), 0);
  EXPECT_THROW(dut.get(0), std::out_of_range);
  EXPECT_THROW(dut.set(0, VectorQD()), std::out_of_range);
  EXPECT_EQ(toEigen(dut), (Eigen::Matrix<double, 0, 1>()));

  dut.append(vq0);
  EXPECT_EQ(dut.count(), 1);
  EXPECT_EQ(dut.size(), 3);
  EXPECT_EQ(dut.get(0).v, vq0.v);
  EXPECT_EQ(toEigen(dut), vq0.v);

  dut.append(vq1);
  EXPECT_EQ(dut.count(), 2);
  EXPECT_EQ(dut.size(), 6);
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
  NAryState<double, VectorQ> dut(2);
  EXPECT_EQ(dut.count(), 2);
  EXPECT_EQ(dut.size(), 6);

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
  NAryState<double, VectorQ> dut((
      Eigen::VectorXd(6) << vq0.v, vq1.v).finished());

  EXPECT_EQ(dut.count(), 2);
  EXPECT_EQ(dut.size(), 6);
  EXPECT_EQ(dut.get(0).v, vq0.v);
  EXPECT_EQ(dut.get(1).v, vq1.v);
  EXPECT_EQ(toEigen(dut),
            (Eigen::VectorXd(6) << vq0.v, vq1.v).finished());

  EXPECT_THROW(dut.get(2), std::out_of_range);
  EXPECT_THROW(dut.set(2, vq1), std::out_of_range);

  // Test assignment from Eigen type.
  dut = vq0.v;
  EXPECT_EQ(dut.count(), 1);
  EXPECT_EQ(dut.size(), 3);
  EXPECT_EQ(dut.get(0).v, vq0.v);
  EXPECT_EQ(toEigen(dut), vq0.v);

  dut = Eigen::VectorXd(0);
  EXPECT_EQ(dut.count(), 0);
  EXPECT_EQ(dut.size(), 0);
  EXPECT_THROW(dut.get(0), std::out_of_range);
  EXPECT_EQ(toEigen(dut), Eigen::VectorXd(0));
}


GTEST_TEST(TestNAryState, NullUnitVectors) {
  NullVector<double> nv;

  // Test default-constructed instance.
  NAryState<double, NullVector> dut;
  EXPECT_EQ(dut.count(), -1);
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.get(0), nv);
  EXPECT_EQ(dut.get(1000), nv);
  dut.set(0, nv);
  dut.set(1000, nv);
  EXPECT_EQ(toEigen(dut), Eigen::VectorXd(0));

  // Appending nothing to nothing should not change nothing.
  dut.append(nv);
  EXPECT_EQ(dut.count(), -1);
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.get(0), nv);
  EXPECT_EQ(dut.get(1000), nv);
  dut.set(0, nv);
  dut.set(1000, nv);
  EXPECT_EQ(toEigen(dut), Eigen::VectorXd(0));

  // Test assignment from Eigen type.
  dut = Eigen::Matrix<double, 0, 1>();
  EXPECT_EQ(dut.count(), -1);
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.get(0), nv);
  EXPECT_EQ(dut.get(1000), nv);
  dut.set(0, nv);
  dut.set(1000, nv);
  EXPECT_EQ(toEigen(dut), Eigen::VectorXd(0));

  // Test construction from Eigen type.
  NAryState<double, NullVector> dut2(Eigen::VectorXd(0));
  EXPECT_EQ(dut2.count(), -1);
  EXPECT_EQ(dut2.size(), 0);
  EXPECT_EQ(dut2.get(0), nv);
  EXPECT_EQ(dut2.get(1000), nv);
  dut2.set(0, nv);
  dut2.set(1000, nv);
  EXPECT_EQ(toEigen(dut2), Eigen::VectorXd(0));
}


}  // namespace
