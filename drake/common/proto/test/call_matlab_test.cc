#include "drake/common/proto/call_matlab.h"

#include <cmath>

#include <gtest/gtest.h>

// Note: Unfortunately these really only test whether the code compiles and
// runs... the actual output must currently be verified by human inspection in
// the matlab window.  See #6707.

namespace drake {
namespace common {

GTEST_TEST(TestCallMatlab, DispEigenMatrix) {
  Eigen::Matrix2d m;
  m << 1, 2, 3, 4;
  CallMatlab("disp", m);

  Eigen::Matrix<bool, 2, 2> b;
  b << true, false, true, false;
  CallMatlab("disp", b);
}

GTEST_TEST(TestCallMatlab, RemoteVarTest) {
  auto magic = CallMatlabSingleOutput("magic", 4);
  CallMatlab("disp", magic);
  CallMatlab("disp", "element(1,1) is ");
  CallMatlab("disp", magic(1, 1));
  CallMatlab("disp", "element(3,2) is ");
  CallMatlab("disp", magic(3, 2));
  CallMatlab("disp", "elements (1:2) are");
  CallMatlab("disp", magic(Eigen::Vector2d(1, 2)));
  CallMatlab("disp", "row 3 is ");
  CallMatlab("disp", magic(3, ":"));
  CallMatlab("disp", "elements (1:5) are");
  CallMatlab("disp", magic(Eigen::VectorXd::LinSpaced(5, 1, 5)));

  CallMatlab("disp", "row 2 (accessed via logicals) is");
  CallMatlab("disp",
             magic(Eigen::Matrix<bool, 4, 1>(false, true, false, false), ":"));

  CallMatlab("disp", "Second column should now be 1,2,3,4: ");
  auto n = magic.subsasgn(Eigen::Vector4d(1, 2, 3, 4), ":", 2);
  CallMatlab("disp", n);
}

GTEST_TEST(TestCallMatlab, SimplePlot) {
  const int N = 100;

  Eigen::VectorXd time(N), val(N);
  for (int i = 0; i < N; i++) {
    time[i] = 0.01 * i;
    val[i] = sin(2 * M_PI * time[i]);
  }

  CallMatlab("disp", "Plotting a (red) sine wave.");
  CallMatlab("figure", 1);
  auto h = CallMatlab(1, "plot", time, val);
  CallMatlab("set", h[0], "Color", "r");
}

GTEST_TEST(TestCallMatlab, MeshTest) {
  const int N = 25, M = 35;
  Eigen::VectorXd x(N), y(M);
  Eigen::MatrixXd Z(M, N);
  for (int i = 0; i < N; i++) {
    x(i) = -3.0 + 6.0 * i / (N - 1);
  }
  for (int i = 0; i < M; i++) {
    y(i) = -3.0 + 6.0 * i / (M - 1);
  }
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < M; j++) {
      Z(j, i) = 3 * pow(1 - x(i), 2) * exp(-pow(x(i), 2) - pow(y(j) + 1, 2)) -
                10 * (x(i) / 5 - pow(x(i), 3) - pow(y(j), 5)) *
                    exp(-pow(x(i), 2) - pow(y(j), 2)) -
                1.0 / 3.0 * exp(-pow(x(i) + 1, 2) - pow(y(j), 2));
    }
  }
  CallMatlab("disp", "Plotting a simple 3D surface");
  CallMatlab("figure", 2);
  CallMatlab("surf", x, y, Z);
}

}  // namespace common
}  // namespace drake
