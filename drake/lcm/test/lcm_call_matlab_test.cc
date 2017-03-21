#include "drake/lcm/lcm_call_matlab.h"

#include <cmath>

#include <gtest/gtest.h>

// Note: Unfortunately these really only test whether the code compiles and
// runs... the actual output must currently be verified by human inspection in
// the matlab window.
// TODO(russt): Add tests using the DrakeMockLcm to verify that the correct
// messages are being sent.

namespace drake {
namespace lcm {

GTEST_TEST(TestLcmCallMatlab, DispEigenMatrix) {
  Eigen::Matrix2d m;
  m << 1, 2, 3, 4;
  LcmCallMatlab("disp", m);

  Eigen::Matrix<bool, 2, 2> b;
  b << true, false, true, false;
  LcmCallMatlab("disp", b);
}

GTEST_TEST(TestLcmCallMatlab, RemoteVarTest) {
  auto magic = LcmCallMatlabSingleOutput("magic", 4);
  LcmCallMatlab("disp", magic);
  LcmCallMatlab("disp", "element(1,1) is ");
  LcmCallMatlab("disp", magic(1, 1));
  LcmCallMatlab("disp", "element(3,2) is ");
  LcmCallMatlab("disp", magic(3, 2));
  LcmCallMatlab("disp", "elements (1:2) are");
  LcmCallMatlab("disp", magic(Eigen::Vector2d(1, 2)));
  LcmCallMatlab("disp", "row 3 is ");
  LcmCallMatlab("disp", magic(3, ":"));
  LcmCallMatlab("disp", "elements (1:5) are");
  LcmCallMatlab("disp", magic(Eigen::VectorXd::LinSpaced(5, 1, 5)));

  LcmCallMatlab("disp", "row 2 (accessed via logicals) is");
  LcmCallMatlab(
      "disp", magic(Eigen::Matrix<bool, 4, 1>(false, true, false, false), ":"));

  LcmCallMatlab("disp", "Second column should now be 1,2,3,4: ");
  auto n = magic.subsasgn(Eigen::Vector4d(1, 2, 3, 4), ":", 2);
  LcmCallMatlab("disp", n);
}

GTEST_TEST(TestLcmCallMatlab, SimplePlot) {
  int N = 100;

  Eigen::VectorXd time(N), val(N);
  for (int i = 0; i < N; i++) {
    time[i] = 0.01 * i;
    val[i] = sin(2 * M_PI * time[i]);
  }

  LcmCallMatlab("disp", "Plotting a (red) sine wave.");
  LcmCallMatlab("figure", 1);
  auto h = LcmCallMatlab(1, "plot", time, val);
  LcmCallMatlab("set", h[0], "Color", "r");
}

GTEST_TEST(TestLcmCallMatlab, MeshTest) {
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
  LcmCallMatlab("disp", "Plotting a simple 3D surface");
  LcmCallMatlab("figure", 2);
  LcmCallMatlab("surf", x, y, Z);
}

}  // namespace lcm
}  // namespace drake
