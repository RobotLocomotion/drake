
#include <typeinfo>
#include "Optimization.h"
#include "testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[])
{
  OptimizationProblem prog;

  auto const &x = prog.addContinuousVariables(4);

  auto x2 = x(2);
  auto xhead = x.head(3);

  Vector4d b = Vector4d::Random();
  prog.addLinearEqualityConstraint(Matrix4d::Identity(),b,{x});
  prog.solve();
  valuecheckMatrix(b,x.value(),1e-10);
  valuecheck(b(2),x2.value()(0),1e-10);
  valuecheckMatrix(b.head(3),xhead.value(),1e-10);

  auto const& y = prog.addContinuousVariables(2);
  prog.addLinearEqualityConstraint(2*Matrix2d::Identity(),b.topRows(2),{y});
  prog.solve();
  valuecheckMatrix(b.topRows(2)/2,y.value(),1e-10);
  valuecheckMatrix(b,x.value(),1e-10);
}