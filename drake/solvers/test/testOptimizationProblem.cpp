
#include <typeinfo>
#include "drake/solvers/Optimization.h"
#include "drake/util/testUtil.h"

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
  void evalImpl(const Ref<const Matrix<ScalarType, Dynamic, 1>>& x, Matrix<ScalarType,Dynamic,1>& y) {
    y.resize(1);
    y(0) = x(0) * x(0) * (4 - 2.1 * x(0) * x(0) + x(0) * x(0) * x(0) * x(0) / 3) + x(0) * x(1) +
           x(1) * x(1) * (-4 + 4 * x(1) * x(1));
  }
};

void sixHumpCamel() {
  OptimizationProblem prog;
  auto x = prog.addContinuousVariables(2);
  auto objective = prog.addCost(make_shared<SixHumpCamelObjective>());
  prog.solve();
  prog.printSolution();

  // check (numerically) if it is a local minimum
  VectorXd ystar, y;
  objective->eval(x.value(),ystar);
  for (int i=0; i<10; i++) {
    objective->eval(x.value() + .01 * Eigen::Matrix<double, 2, 1>::Random(), y);
    if (y(0)<ystar(0)) throw std::runtime_error("not a local minima!");
  }
}

class GloptipolyConstrainedExampleObjective
        : public TemplatedDifferentiableFunction<GloptipolyConstrainedExampleObjective> {
public:
  GloptipolyConstrainedExampleObjective() : TemplatedDifferentiableFunction<GloptipolyConstrainedExampleObjective>(*this) {};

  template<typename ScalarType>
  void evalImpl(const Ref<const Matrix<ScalarType, Dynamic, 1>>& x, Matrix<ScalarType,Dynamic,1>& y) const {
    y.resize(1);
    y(0) = -2*x(0) + x(1) - x(2);
  }
};

class GloptipolyConstrainedExampleConstraint : public Constraint {  // want to also support deriving directly from constraint without going through Drake::Function
public:
  GloptipolyConstrainedExampleConstraint(const VariableList& vars) : Constraint(vars, Vector1d::Constant(0), Vector1d::Constant(numeric_limits<double>::infinity())) {}

  // for just these two types, implementing this locally is almost cleaner...
  virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const override { evalImpl(x,y); }
  virtual void eval(const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const override { evalImpl(x,y); }

  template<typename ScalarType>
  void evalImpl(const Ref<const Matrix<ScalarType, Dynamic, 1>>& x, Matrix<ScalarType,Dynamic,1>& y) const {
    y.resize(1);
    y(0) = 24 - 20*x(0) + 9*x(1) - 13*x(2) + 4*x(0)*x(0) - 4*x(0)*x(1) + 4*x(0)*x(2) + 2*x(1)*x(1) - 2*x(1)*x(2) + 2*x(2)*x(2);
  }
};

/** gloptiPolyConstrainedMinimization
 * @brief from section 5.8.2 of the gloptipoly3 documentation
 *
 * Which is from section 3.5 in
 *   Handbook of Test Problems in Local and Global Optimization
 */
void gloptipolyConstrainedMinimization() {
  OptimizationProblem prog;
  auto x = prog.addContinuousVariables(3);
  prog.addCost(make_shared<GloptipolyConstrainedExampleObjective>());
  std::shared_ptr<GloptipolyConstrainedExampleConstraint> qp_con(new GloptipolyConstrainedExampleConstraint({x}));
  prog.addConstraint(qp_con);
  prog.addLinearConstraint(Vector3d(1,1,1).transpose(),Vector1d::Constant(-numeric_limits<double>::infinity()),Vector1d::Constant(4));
  prog.addLinearConstraint(Vector3d(0,3,1).transpose(),Vector1d::Constant(-numeric_limits<double>::infinity()),Vector1d::Constant(6));
  prog.addBoundingBoxConstraint(Vector3d(0,0,0),Vector3d(2,numeric_limits<double>::infinity(),3));

  prog.setInitialGuess({x},Vector3d(.5,0,3)+.1*Vector3d::Random());
  prog.solve();
  prog.printSolution();

  valuecheckMatrix(x.value(),Vector3d(.5,0,3),1e-4);
}

int main(int argc, char* argv[])
{
  trivialLeastSquares();
  sixHumpCamel();
  gloptipolyConstrainedMinimization();
  return 0;
}