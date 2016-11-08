#include <cmath>

#include "gtest/gtest.h"

#include "drake/lcm/lcm_call_matlab.h"

// Test that linearizing an affine system returns the original A,B,C,D matrices.
GTEST_TEST(TestLcmCallMatlab, SimplePlot) {
  int N = 100;

  Eigen::VectorXd time(N), val(N);
  for (int i=0; i<N; i++) {
    time[i] = 0.01*i;
    val[i] = sin(2*M_PI*time[i]);
  }

  auto h = lcm_call_matlab("plot",1,time,val);
//  lcm_call_matlab("set",0,h[0],"Color","r");
  lcm_call_matlab("disp",0,"Hello World!");
  lcm_call_matlab("drawnow",0);
}
