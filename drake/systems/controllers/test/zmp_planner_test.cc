#include <iostream>
#include <fstream>

#include "drake/systems/controllers/zmp_planner.h"

int main() {
  drake::systems::ZMPPlanner zmp;

  std::vector<double> Ts;
  std::vector<Eigen::MatrixXd> zmp_d;
  std::ofstream out;

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

  for (const auto& knot : zmp_d) {
    std::cout << knot(1,0) << std::endl;
  }

  for (const auto& t : Ts) {
    std::cout << t << std::endl;
  }

  //PiecewisePolynomial<double> zmp_traj_ = PiecewisePolynomial<double>::FirstOrderHold(Ts, zmp_d);
  PiecewisePolynomial<double> zmp_traj_ = PiecewisePolynomial<double>::Pchip(Ts, zmp_d);
  for (int i = 0; i < zmp_traj_.getNumberOfSegments(); i++) {
    std::cout << zmp_traj_.getPolynomialMatrix(i)(1, 0).GetCoefficients().transpose() << std::endl;
  }

  zmp.Plan(zmp_traj_, x0, 1);

  //std::cout << "S: " << zmp.S_ << std::endl;
  out.open("/home/sfeng/zmp_d");
  for (double t = Ts[0]; t <= Ts[Ts.size()-1] + 5; t+= 0.01) {
    out << t << " " << zmp.get_desired_zmp(t).transpose() << std::endl;
  }
  out.close();

  out.open("/home/sfeng/com");
  for (double t = Ts[0]; t <= Ts[Ts.size()-1] + 5; t+= 0.01) {
    out << t << " " << zmp.get_nominal_com(t).transpose() << std::endl;
  }
  out.close();

  out.open("/home/sfeng/ff");
  for (double t = Ts[0]; t <= Ts[Ts.size()-1] + 5; t+= 0.01) {
    Eigen::Vector4d nominal_x;
    nominal_x << zmp.get_nominal_com(t), zmp.get_nominal_comd(t);
    out << t << " " << zmp.get_nominal_comdd(t).transpose() << " " << zmp.ComputeOptimalCoMdd(t, nominal_x).transpose() << std::endl;
  }
  out.close();

  return 0;
}
