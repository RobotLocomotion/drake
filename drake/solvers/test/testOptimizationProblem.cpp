
#include <typeinfo>
#include "Optimization.h"
#include "testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

void trivialLeastSquares() {
  OptimizationProblem prog;

  auto const &x = prog.addContinuousVariables(4);

  auto x2 = x(2);
  auto xhead = x.head(3);

  Vector4d b = Vector4d::Random();
  auto con = prog.addLinearEqualityConstraint(Matrix4d::Identity(), b, {x});
  prog.solve();
  valuecheckMatrix(b, x.value(), 1e-10);
  valuecheck(b(2), x2.value()(0), 1e-10);
  valuecheckMatrix(b.head(3), xhead.value(), 1e-10);
  valuecheck(b(2), xhead(2).value()(0), 1e-10); // a segment of a segment

  auto const &y = prog.addContinuousVariables(2);
  prog.addLinearEqualityConstraint(2 * Matrix2d::Identity(), b.topRows(2), {y});
  prog.solve();
  valuecheckMatrix(b.topRows(2) / 2, y.value(), 1e-10);
  valuecheckMatrix(b, x.value(), 1e-10);

  con->updateConstraint(3 * Matrix4d::Identity(), b);
  prog.solve();
  valuecheckMatrix(b.topRows(2) / 2, y.value(), 1e-10);
  valuecheckMatrix(b / 3, x.value(), 1e-10);

  std::shared_ptr<BoundingBoxConstraint> bbcon(
          new BoundingBoxConstraint({x.head(2)}, MatrixXd::Constant(2, 1, -1000.0), MatrixXd::Constant(2, 1, 1000.0)));
  prog.addConstraint(bbcon);
  prog.solve();  // now it will solve as a nonlinear program
  valuecheckMatrix(b.topRows(2) / 2, y.value(), 1e-10);
  valuecheckMatrix(b / 3, x.value(), 1e-10);
}


class SixHumpCamelObjective : public TemplatedDifferentiableFunction<SixHumpCamelObjective> {
public:
  SixHumpCamelObjective() : TemplatedDifferentiableFunction<SixHumpCamelObjective>(*this) {};

  template<typename ScalarType>
  Matrix<ScalarType,1,1> eval(const Ref<Matrix<ScalarType, 2, 1>> &x) {
    return Matrix<ScalarType,1,1>::Constant(x(0) * x(0) * (4 - 2.1 * x(0) * x(0) + x(0) * x(0) * x(0) * x(0) / 3) + x(0) * x(1) +
           x(1) * x(1) * (-4 + 4 * x(1) * x(1)));
  }
};

void sixHumpCamel() {
  OptimizationProblem prog;
  auto x = prog.addContinuousVariables(2);
  auto objective = make_shared<FunctionConstraint<SixHumpCamelObjective>({x});
  prog.addObjective(objective);
  prog.solve();
  prog.printSolution();
  // todo: check that it's one of the six local minima?
}

int main(int argc, char* argv[])
{
  trivialLeastSquares();
  sixHumpCamel();
  return 0;
}