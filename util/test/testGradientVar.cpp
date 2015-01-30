#include <Eigen/Core>
#include "GradientVar.h"
#include "testUtil.h"
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

void test()
{
  const int rows = 2;
  const int cols = 3;

  GradientVar<double, rows, cols> var1(rows, cols);
  var1.value().setConstant(1.0);
  std::cout << var1.value() << std::endl << std::endl;
  std::cout << "maxOrder: " << var1.maxOrder() << std::endl << std::endl;

  const int nq = 5;
  GradientVar<double, rows, cols> var2(rows, cols, nq, 1);
  var2.value().setConstant(2.0);
  std::cout << var2.value() << std::endl << std::endl;
  var2.gradient().value().setConstant(3.0);
  std::cout << var2.gradient().value() << std::endl << std::endl;
  std::cout << "maxOrder: " << var2.maxOrder() << std::endl << std::endl;

  GradientVar<double, rows, cols> var3(rows, cols, nq, 5);
//  std::cout << "var3 rows: " << var3.secondDerivative().rows() << ", cols: " << var3.secondDerivative().cols() << std::endl;
  var3.gradient().gradient().value().setConstant(4.0);
  std::cout << var3.gradient().gradient().value() << std::endl << std::endl;
  std::cout << "maxOrder: " << var3.maxOrder() << std::endl << std::endl;

  GradientVar<double, 6, Eigen::Dynamic> var4(6, 3);
  var4.value().setConstant(5.0);
  std::cout << var4.value() << std::endl << std::endl;

  GradientVar<double, rows, cols> var3_copy(var3);
  std::cout << var3_copy.gradient().gradient().value() << std::endl << std::endl;
}

int main(int argc, char **argv)
{
  test();

  return 0;
}
