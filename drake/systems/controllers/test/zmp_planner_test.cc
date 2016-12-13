#include <iostream>
#include <fstream>

#include "drake/systems/controllers/zmp_planner.h"

void TestZMPPlanner(const PiecewisePolynomial<double>& zmp_d,
    const Eigen::Vector4d& x0, double height) {
  std::ofstream out;
  drake::systems::ZMPPlanner zmp_planner;
  zmp_planner.Plan(zmp_d, x0, height);

  out.open("/home/sfeng/zmp_d");
  for (double t = zmp_d.getStartTime(); t <= zmp_d.getEndTime() + 1; t+= 0.01) {
    out << t << " " << zmp_planner.get_desired_zmp(t).transpose() << std::endl;
  }
  out.close();

  out.open("/home/sfeng/com");
  for (double t = zmp_d.getStartTime(); t <= zmp_d.getEndTime() + 1; t+= 0.01) {
    out << t << " " << zmp_planner.get_nominal_com(t).transpose() << std::endl;
  }
  out.close();

  out.open("/home/sfeng/ff");
  for (double t = zmp_d.getStartTime(); t <= zmp_d.getEndTime() + 1; t+= 0.01) {
    Eigen::Vector4d nominal_x;
    nominal_x << zmp_planner.get_nominal_com(t),
                 zmp_planner.get_nominal_comd(t);
    out << t << " " << zmp_planner.get_nominal_comdd(t).transpose() << " "
        << zmp_planner.ComputeOptimalCoMdd(t, nominal_x).transpose()
        << std::endl;
  }
  out.close();
}

int main() {
  std::vector<double> Ts;
  std::vector<Eigen::MatrixXd> zmp_d;

  Eigen::Vector4d x0(0, 0, 0, 0);
  double x = 0;
  double y = 0.5;
  double time = 0;
  double ss = 1;
  double ds = 0.5;
  Ts.push_back(time);
  zmp_d.push_back(x0.head<2>());

  for (int i = 0; i < 5; i++) {
    time += ds;
    x += 0.5;
    y *= -1;
    Ts.push_back(time);
    zmp_d.push_back(Eigen::Vector2d(x, y));

    time += ss;
    Ts.push_back(time);
    zmp_d.push_back(Eigen::Vector2d(x, y));
  }

  PiecewisePolynomial<double> zmp_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(Ts, zmp_d);
  // PiecewisePolynomial<double> zmp_traj =
  //  PiecewisePolynomial<double>::FirstOrderHold(Ts, zmp_d);
  // PiecewisePolynomial<double> zmp_traj =
  //  PiecewisePolynomial<double>::Pchip(Ts, zmp_d);

  double z = 1;
  TestZMPPlanner(zmp_traj, x0, z);

  return 0;
}
