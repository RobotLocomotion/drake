#include <Eigen/Core>
#include <iostream>

using namespace Eigen;
using namespace std;

template <int i>
Matrix<double, 6, Dynamic> bla()
{
  return Matrix<double, 6, i>::Random();
}

void testRows() {
  MatrixXd m(3,3);
  m << 1,2,3,
       4,5,6,
       7,8,9;

  MatrixXd a(3,3);
  a << 1,2,3,
       4,5,6,
       7,8,9;

  cout << "Here is the matrix m:" << endl << m << endl;
  cout << "2nd Row: " << m.row(1) << endl;
  m.row(2) += 3 * a.row(0);
  cout << "After adding 3 times the first row into the third row, the matrix m is:\n";
  cout << m << endl;
}

int main(int argc, char **argv) {
//  typedef Matrix<double, 6, 1> Vector6d;
//  typedef Matrix<double, 6, 6> Matrix6d;
//  auto mat = bla<3>();
//  std::cout << mat.RowsAtCompileTime << std::endl << std::endl;

  testRows();

  return 0;
}
