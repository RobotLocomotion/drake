
#include "Optimization.h"
#include <typeinfo>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[])
{
  OptimizationProblem prog;

  auto const& x = prog.addContinuousVariables(4);

  Vector4d b = Vector4d::Random();
  prog.addLinearEqualityConstraint(Matrix4d::Identity(),b,x);
  prog.solve();
  assert((b - x.value).isZero());

  auto const& y = prog.addContinuousVariables(2);
  prog.addLinearEqualityConstraint(2*Matrix2d::Identity(),b.topRows(2),y);
  prog.solve();
  assert((b.topRows(2)/2 - y.value).isZero());

}