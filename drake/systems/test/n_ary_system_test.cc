#include "drake/systems/n_ary_system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/core/Vector.h"
#include "drake/systems/cascade_system.h"

namespace {

using Drake::NullVector;
using Drake::toEigen;
using drake::NAryState;
using drake::NArySystem;

// Vector-concept class for exercising composition.
template <typename ScalarType>
struct StateQ {
  static const int RowsAtCompileTime = Eigen::Dynamic;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  StateQ() {}

  StateQ(const double a, const double b)
      : v((Eigen::VectorXd(2) << a, b).finished()) {}

  template <typename Derived>
  explicit StateQ(const Eigen::MatrixBase<Derived>& initial)
      : v(initial) {}

  template <typename Derived>
  StateQ& operator=(const Eigen::MatrixBase<Derived>& rhs) {
    v = rhs;
  }

  friend EigenType toEigen(const StateQ<ScalarType>& s) {
    return s.v;
  }

  std::size_t size() const { return 2; }

  EigenType v;
};

using StateQD = StateQ<double>;

// System-concept class for exercising composition.
struct SystemQ {
  template <typename ScalarType>
  using StateVector = StateQ<ScalarType>;
  template <typename ScalarType>
  using InputVector = StateQ<ScalarType>;
  template <typename ScalarType>
  using OutputVector = StateQ<ScalarType>;

  SystemQ(const double dynamics_magic, const double output_magic) :
      dynamics_magic_(dynamics_magic, 0.),
      output_magic_(output_magic, 0.) {}

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& time,
                                   const StateVector<ScalarType>& state,
                                   const InputVector<ScalarType>& input) const {
    return StateVector<ScalarType>(state.v + input.v + dynamics_magic_.v);
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>& state,
                                  const InputVector<ScalarType>& input) const {
    return OutputVector<ScalarType>(state.v + input.v + output_magic_.v);
  }

  bool isTimeVarying() const { return time_varying_; }

  bool isDirectFeedthrough() const { return feedthrough_; }

  const StateQ<double> dynamics_magic_;
  const StateQ<double> output_magic_;
  bool time_varying_ { false };
  bool feedthrough_ { false };
};


// Conveniently makes a 4-d VectorXd.
Eigen::VectorXd Make4d(double c1, double c2, double c3, double c4) {
  return (Eigen::VectorXd(4) << c1, c2, c3, c4).finished();
}


GTEST_TEST(TestNArySystem, BasicOperation) {
  std::shared_ptr<SystemQ> sq1 = std::make_shared<SystemQ>(10., 100.);
  std::shared_ptr<SystemQ> sq2 = std::make_shared<SystemQ>(20., 200.);

  NArySystem<SystemQ> dut;
  NAryState<StateQ<double> > state;
  NAryState<StateQ<double> > input;

  EXPECT_EQ(dut.isTimeVarying(), false);
  EXPECT_EQ(dut.isDirectFeedthrough(), false);
  EXPECT_EQ(dut.getNumStates(), 0u);
  EXPECT_EQ(dut.getNumInputs(), 0u);
  EXPECT_EQ(dut.getNumOutputs(), 0u);
  EXPECT_EQ(dut.dynamics(0., state, input).count(), 0);
  EXPECT_EQ(dut.output(0., state, input).count(), 0);

  dut.AddSystem(sq1);
  EXPECT_EQ(dut.isTimeVarying(), false);
  EXPECT_EQ(dut.isDirectFeedthrough(), false);
  EXPECT_EQ(dut.getNumStates(), 2u);
  EXPECT_EQ(dut.getNumInputs(), 2u);
  EXPECT_EQ(dut.getNumOutputs(), 2u);
  EXPECT_THROW(dut.dynamics(0., state, input), std::invalid_argument);
  EXPECT_THROW(dut.output(0., state, input), std::invalid_argument);
  StateQ<double> s1(1., 0.);
  StateQ<double> i1(5., 0.);
  state.Append(s1);
  input.Append(i1);
  EXPECT_EQ(dut.dynamics(0., state, input).count(), 1);
  EXPECT_EQ(dut.output(0., state, input).count(), 1);

  dut.AddSystem(sq2);
  EXPECT_EQ(dut.isTimeVarying(), false);
  EXPECT_EQ(dut.isDirectFeedthrough(), false);
  EXPECT_EQ(dut.getNumStates(), 4u);
  EXPECT_EQ(dut.getNumInputs(), 4u);
  EXPECT_EQ(dut.getNumOutputs(), 4u);
  EXPECT_THROW(dut.dynamics(0., state, input), std::invalid_argument);
  EXPECT_THROW(dut.output(0., state, input), std::invalid_argument);
  StateQ<double> s2(2., 0.);
  StateQ<double> i2(6., 0.);
  state.Append(s2);
  input.Append(i2);
  EXPECT_EQ(dut.dynamics(0., state, input).count(), 2);
  EXPECT_EQ(dut.output(0., state, input).count(), 2);

  EXPECT_EQ(toEigen(dut.dynamics(0., state, input)),
            Make4d(10. + 1. + 5., 0.,
                   20. + 2. + 6., 0.));
  EXPECT_EQ(toEigen(dut.output(0., state, input)),
            Make4d(100. + 1. + 5., 0.,
                   200. + 2. + 6., 0.));

  sq2->time_varying_ = true;
  sq2->feedthrough_ = true;
  EXPECT_EQ(dut.isTimeVarying(), true);
  EXPECT_EQ(dut.isDirectFeedthrough(), true);
}


// Ensure that cascade() will compile.
GTEST_TEST(TestNArySystem, Cascade) {
  std::shared_ptr<SystemQ> sq1 = std::make_shared<SystemQ>(10., 100.);
  std::shared_ptr<SystemQ> sq2 = std::make_shared<SystemQ>(20., 200.);

  auto n1 = std::make_shared<NArySystem<SystemQ> >();
  auto n2 = std::make_shared<NArySystem<SystemQ> >();
  auto dut = Drake::cascade(n1, n2);
}

}  // namespace
