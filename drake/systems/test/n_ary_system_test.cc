#include "drake/systems/n_ary_system.h"

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>

#include "drake/core/Vector.h"
#include "drake/systems/cascade_system.h"

#include "gtest/gtest.h"

namespace {

using Drake::NullVector;
using Drake::toEigen;
using drake::NAryState;
using drake::NArySystem;

// Vector-concept class for exercising composition.
template <typename ScalarType>
class StateQ {
 public:
  static const int RowsAtCompileTime = 2;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  StateQ() {}

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

  EigenType v;
};

using StateQD = StateQ<double>;

// System-concept class for exercising composition.
class SystemQ {
 public:
  template <typename ScalarType>
  using StateVector = StateQ<ScalarType>;
  template <typename ScalarType>
  using InputVector = StateQ<ScalarType>;
  template <typename ScalarType>
  using OutputVector = StateQ<ScalarType>;

  SystemQ(const double dynamics_magic, const double output_magic) :
      dynamics_magic_(Eigen::Vector2d(dynamics_magic, 0.)),
      output_magic_(Eigen::Vector2d(output_magic, 0.)) {}

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

  bool isTimeVarying() const { return true; }

  bool isDirectFeedthrough() const { return true; }

 private:
  const StateQ<double> dynamics_magic_;
  const StateQ<double> output_magic_;
};


GTEST_TEST(TestNArySystem, BasicOperation) {
  std::shared_ptr<SystemQ> sq1 = std::make_shared<SystemQ>(10., 100.);
  std::shared_ptr<SystemQ> sq2 = std::make_shared<SystemQ>(20., 200.);

  NArySystem<SystemQ> dut;
  NAryState<double, StateQ> state;
  NAryState<double, StateQ> input;
  EXPECT_EQ(dut.isTimeVarying(), false);
  EXPECT_EQ(dut.isDirectFeedthrough(), false);
  EXPECT_EQ(dut.getNumStates(), 0);
  EXPECT_EQ(dut.getNumInputs(), 0);
  EXPECT_EQ(dut.getNumOutputs(), 0);
  EXPECT_EQ(dut.dynamics(0., state, input).count(), 0);
  EXPECT_EQ(dut.output(0., state, input).count(), 0);

  dut.addSystem(sq1);
  EXPECT_EQ(dut.isTimeVarying(), true);
  EXPECT_EQ(dut.isDirectFeedthrough(), true);
  EXPECT_EQ(dut.getNumStates(), 2);
  EXPECT_EQ(dut.getNumInputs(), 2);
  EXPECT_EQ(dut.getNumOutputs(), 2);
  EXPECT_THROW(dut.dynamics(0., state, input), std::invalid_argument);
  EXPECT_THROW(dut.output(0., state, input), std::invalid_argument);
  StateQ<double> s1(Eigen::Vector2d(1., 0.));
  StateQ<double> i1(Eigen::Vector2d(5., 0.));
  state.append(s1);
  input.append(i1);
  EXPECT_EQ(dut.dynamics(0., state, input).count(), 1);
  EXPECT_EQ(dut.output(0., state, input).count(), 1);

  dut.addSystem(sq2);
  EXPECT_EQ(dut.getNumStates(), 4);
  EXPECT_EQ(dut.getNumInputs(), 4);
  EXPECT_EQ(dut.getNumOutputs(), 4);
  EXPECT_THROW(dut.dynamics(0., state, input), std::invalid_argument);
  EXPECT_THROW(dut.output(0., state, input), std::invalid_argument);
  StateQ<double> s2(Eigen::Vector2d(2., 0.));
  StateQ<double> i2(Eigen::Vector2d(6., 0.));
  state.append(s2);
  input.append(i2);
  EXPECT_EQ(dut.dynamics(0., state, input).count(), 2);
  EXPECT_EQ(dut.output(0., state, input).count(), 2);

  EXPECT_EQ(toEigen(dut.dynamics(0., state, input)),
            Eigen::Vector4d(10. + 1. + 5., 0.,
                            20. + 2. + 6., 0.));
  EXPECT_EQ(toEigen(dut.output(0., state, input)),
            Eigen::Vector4d(100. + 1. + 5., 0.,
                            200. + 2. + 6., 0.));
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
