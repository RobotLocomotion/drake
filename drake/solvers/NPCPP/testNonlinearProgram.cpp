#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "NonlinearProgram.h"
#include "LinearConstraint.h"
#include "QuadraticConstraint.h"
#include "FunctionHandleConstraint.h"
#include "BoundingBoxConstraint.h"
#include "ConstantConstraint.h"
#include "Constraint.h"

namespace snopt {
#include "snopt.hh"
}

using namespace std;
using namespace snopt;
using namespace drake;

#define INF 1.1e+20

void testNonlinearProgram_linearFunction() {
  /**
   * Solves:
   * min    x2
   * st     x1 + x2 = 10
   *        1 <= x1 <= 9
   *
   * Otherwise known as minimum of y = -x + 10, 1 <= x <= 9
   */
  integer n = 2;
  NonlinearProgram np(n);

  unique_ptr<Constraint> cost(new BoundingBoxConstraint(-INF, INF, n, 2));

  vector<pair<integer, doublereal>> A_constr1(2);
  A_constr1[0] = make_pair(1, 1.0);
  A_constr1[1] = make_pair(2, 1.0);
  unique_ptr<Constraint> constr1(new LinearConstraint(10, 10, n, A_constr1));

  unique_ptr<Constraint> constr2(new BoundingBoxConstraint(1, 9, n, 1));

  np.setCost(cost);
  np.addConstraint(constr1);
  np.addConstraint(constr2);

  vector<doublereal> x(n);
  doublereal objval;
  integer info;
  np.solve(&x, &objval, &info);

  printf("%s", "==========\nLinear Problem\n\n");
  printf("Value of x: %e\nValue of y: %e\nObjective Value: %e\nInfo: %ld\n", x[0], x[1], objval, info);
}


void testNonlinearProgram_quadraticFunction() {
  /**
   * Solves:
   * min    x2
   * st     x1^2 - 6*x1 - x2 = -9
   *
   * Otherwise known as minimum of y = x^2 -6x + 9
   */
   
  integer n = 2;
  NonlinearProgram np(n);

  unique_ptr<Constraint> cost(new BoundingBoxConstraint(-INF, INF, n, 2));

  vector<pair<integer, doublereal>> Q_constr(1, make_pair(1, 1.0));
  vector<pair<integer, doublereal>> b_constr(1, make_pair(1, -6.0));
  vector<pair<integer, doublereal>> A_constr(1, make_pair(2, -1.0));
  unique_ptr<Constraint> constr(new QuadraticConstraint(
    -9.0, -9.0, n, Q_constr, b_constr, A_constr));

  np.setCost(cost);
  np.addConstraint(constr);

  vector<doublereal> x(n);
  doublereal objval;
  integer info;
  np.solve(&x, &objval, &info);

  printf("%s", "==========\nQuadratic Problem\n\n");
  printf("Value of x: %e\nValue of y: %e\nObjective Value: %e\nInfo: %ld\n", x[0], x[1], objval, info);
}

void testNonlinearProgram_exponentialFunction() {
  /**
   * Solves:
   * min     x2
   * st      4*exp(x1) - x2 = 0
   *         -1 <= x1 <= 1
   *
   *         Otherwise known as minimum of y = 4exp(x); -1 <= x <= 1
   */
  integer n = 2;
  NonlinearProgram np(n);

  unique_ptr<Constraint> cost(new BoundingBoxConstraint(-INF, INF, n, 2));

  vector<pair<integer, doublereal>> A_constr1(1, make_pair(2, -1.0));
  function<void(doublereal[],bool,bool,doublereal*,std::vector<doublereal>*)> fun_constr1;
  fun_constr1 = [](doublereal x[],
      bool needF,
      bool needG,
      doublereal* f,
      std::vector<doublereal>* g) -> void {
    if (needF) {
      *f = 4 * exp(x[0]);
    }
    if (needG) {
      g->clear();
      g->push_back(4 * exp(x[0]));
    }
  };
  vector<integer> jGvar_constr1(1, 1);
  unique_ptr<Constraint> constr1(new FunctionHandleConstraint(0, 0, n, A_constr1, jGvar_constr1, fun_constr1));

  unique_ptr<Constraint> constr2(new BoundingBoxConstraint(-1, 1, n, 1));

  np.setCost(cost);
  np.addConstraint(constr1);
  np.addConstraint(constr2);

  vector<doublereal> x(n);
  doublereal objval;
  integer info;
  np.solve(&x, &objval, &info);

  printf("%s", "==========\nExponential Problem\n\n");
  printf("Value of x: %e\nValue of y: %e\nObjective Value: %e\nInfo: %ld\n", x[0], x[1], objval, info);
}

int main() {
  testNonlinearProgram_linearFunction();
  testNonlinearProgram_quadraticFunction();
  testNonlinearProgram_exponentialFunction();

  return 0;
}
