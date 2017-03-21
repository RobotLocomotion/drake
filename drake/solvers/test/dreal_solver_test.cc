#include "drake/solvers/dreal_solver.h"

#include <typeinfo>

#include <gtest/gtest.h>
#include "dreal/dreal.h"

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {

// TODO(soonho-tri): Replace this test with ones testing DrealSolver
// class. For now, it simply checks whether dReal is properly linked.
GTEST_TEST(testDreal, example1) {
  using dreal::solver;
  using dreal::expr;
  using dreal::vtype;
  solver s;
  expr const x = s.var("x", vtype::Real);
  expr const zero = s.num(0.0);
  expr const sn = sin(x);
  expr const phi = (sn == zero);
  expr const f = x + x * x + sin(x * sin(x));
  expr const phi2 = (-f == 0);
  s.add(phi);
  s.add(phi2);
  EXPECT_TRUE(s.check());
}
}  // close namespace
}  // close namespace solvers
}  // close namespace drake
