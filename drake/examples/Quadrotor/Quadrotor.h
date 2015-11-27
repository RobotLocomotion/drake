#ifndef _QUADROTOR_H_
#define _QUADROTOR_H_

#include <iostream>
#include <cmath>
#include "DrakeSystem.h"
#include "drakeGeometryUtil.h"
#include "LCMCoordinateFrame.h"
#include "BotVisualizer.h"

class Quadrotor : public DrakeSystem {
public:
  Quadrotor(const std::shared_ptr<lcm::LCM>& lcm) :
          DrakeSystem("Quadrotor",
                  std::make_shared<CoordinateFrame>("QuadrotorContState", std::vector<std::string>({"x", "y", "z", "roll", "pitch", "yaw", "xdot", "ydot", "zdot", "rolldot", "pitchdot", "yawdot"})),
                  nullptr,
                  std::make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("QuadrotorInput", std::vector<std::string>({"u1, u2, u3, u4"}), lcm),
                  std::make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("QuadrotorState", std::vector<std::string>({"x", "y", "z", "roll", "pitch", "yaw", "xdot", "ydot", "zdot", "rolldot", "pitchdot", "yawdot"}), lcm)),
            m(1.0),
            g(9.81),
            L(0.1750),
            I(Eigen::Matrix3d::Identity()),
            num_states(getStateFrame().getDim()),
            num_inputs(getInputFrame().getDim())
  {
  }
  virtual ~Quadrotor(void) {};

  virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return dynamics_implementation(t,x,u);
  }

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return x;
  }

  virtual bool isTimeInvariant() const override { return true; }
  virtual bool isDirectFeedthrough() const override { return false; }


  DrakeSystemPtr balanceLQR() {
    Eigen::MatrixXd Q(num_states, num_states);  Q = Eigen::MatrixXd::Identity(num_states, num_states);
    Eigen::MatrixXd R(num_inputs, 1);  R << 1;
    Eigen::VectorXd xG(num_states);   xG << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd uG(num_inputs);   uG = Eigen::VectorXd::Ones(4) * m * g * 0.25;

    return timeInvariantLQR(xG,uG,Q,R);
  }

  double g, L, m;
  Eigen::Matrix3d I;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  const unsigned int num_states;
  const unsigned int num_inputs;

  template <typename Scalar, typename Vector>
  Vector dynamics_implementation(Scalar t, const Vector& x, const Vector& u) const {
    Vector xdot(num_states);
    xdot = Vector::Zero(num_states);

    double phi = x(3);
    double theta = x(4);
    double psi = x(5);
    double phidot = x(9);
    double thetadot = x(10);
    double psidot = x(11);

    double w1 = u(0);
    double w2 = u(1);
    double w3 = u(2);
    double w4 = u(3);


    Eigen::Vector3d rpy;
    rpy << phi, theta, psi;
    Eigen::Matrix3d R = rpy2rotmat(rpy);

    const double kf = 1;
    const double km = 0.0245;
    
    double F1 = kf * w1;
    double F2 = kf * w2;
    double F3 = kf * w3;
    double F4 = kf * w4;

    double M1 = km * w1;
    double M2 = km * w2;
    double M3 = km * w3;
    double M4 = km * w4;

    Eigen::Vector3d gvec;
    gvec << 0, 0, -m * g;
    Eigen::Vector3d forcevec;
    forcevec << 0, 0, F1 + F2 + F3 + F4;
    Eigen::Vector3d xyz_ddot = (1.0 / m) * (gvec + R * forcevec);

    Eigen::Vector3d rpydot;
    rpydot << phidot, thetadot, psidot;

    Eigen::Vector3d pqr;
    rpydot2angularvel(rpy, rpydot, pqr);
    pqr = R.adjoint() * pqr;

    Eigen::Vector3d pqr_dot_term1;
    pqr_dot_term1 << L * (F2 - F4), L * (F3 - F1), (M1 - M2 + M3 - M4);
    Eigen::Vector3d pqr_dot = I.ldlt().solve(pqr_dot_term1 - pqr.cross(I * pqr));

    Eigen::Matrix3d Phi;
    Gradient<Eigen::Matrix3d, 3>::type dPhi;
    auto ddPhi = (Gradient<Eigen::Matrix3d, 3, 2>::type*) nullptr;
    angularvel2rpydotMatrix(rpy, Phi, &dPhi, ddPhi);

    // This replaces the implementation of Rdot (make a 9x1 vector, reshape it into a 3 x 3)
    Eigen::Matrix<double, 9, 3> drpy2drotmat = drpy2rotmat(rpy);
    Eigen::Matrix<double, 9, 1> Rdot_vec;
    Rdot_vec = drpy2drotmat * rpydot;
    Eigen::Matrix3d Rdot = Eigen::Map<Eigen::MatrixXd>(Rdot_vec.data(), 3, 3);

    Eigen::Matrix<double, 9, 1> dPhi_x_rpydot_vec;
    dPhi_x_rpydot_vec = dPhi * rpydot;
    Eigen::Matrix3d dPhi_x_rpydot = Eigen::Map<Eigen::MatrixXd>(dPhi_x_rpydot_vec.data(), 3, 3);
    Eigen::Vector3d rpy_ddot = Phi * R * pqr_dot + dPhi_x_rpydot * R * pqr + Phi * Rdot * pqr;

    Eigen::Matrix<double, 6, 1> qdd;
    qdd << xyz_ddot, rpy_ddot;
    Eigen::Matrix<double, 6, 1> qd;
    qd = x.template segment<6>(6);

    xdot << qd, qdd;

    return xdot;
  }

};

class QuadrotorWithBotVis : public Quadrotor {
public:
  QuadrotorWithBotVis(const std::shared_ptr<lcm::LCM>& lcm) 
  : Quadrotor(lcm), botvis(lcm, "Quadrotor.urdf", DrakeJoint::ROLLPITCHYAW) {}

  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    botvis.output(t,Eigen::VectorXd::Zero(0),x);
    return Quadrotor::output(t,x,u);
  }

  BotVisualizer botvis;
};

#endif // _QUADROTOR_H_
