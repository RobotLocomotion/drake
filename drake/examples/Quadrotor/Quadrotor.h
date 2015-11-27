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
  {}
  virtual ~Quadrotor(void) {};

  virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return dynamics_implementation(t, x, u);
  }
  
  virtual Drake::TaylorVecX dynamics(Drake::TaylorVarX t, const Drake::TaylorVecX& x, const Drake::TaylorVecX& u) const override {
    return dynamics_implementation(t, x, u);
  }
  
  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return x;
  }

  virtual bool isTimeInvariant() const override { return true; }
  virtual bool isDirectFeedthrough() const override { return false; }

  DrakeSystemPtr balanceLQR() {
    const int num_positions = num_states/2;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(num_states, num_states);
    Q.block<0, 0>(num_positions, num_positions) = 10.0 * Eigen::MatrixXd::Ones(num_positions, num_positions);
    Eigen::VectorXd R = Eigen::VectorXd::Ones(num_inputs);
    Eigen::VectorXd xG(num_states);   xG << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd uG = Eigen::VectorXd::Ones(num_inputs) * m * g * 0.25;

    return timeInvariantLQR(xG,uG,Q,R);
  }

  double g, L, m;
  Eigen::Matrix3d I;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  const unsigned int num_states;
  const unsigned int num_inputs;
  const double kf = 1;
  const double km = 0.0245;

  template <typename Scalar, typename Vector>
  Vector dynamics_implementation(Scalar t, const Vector& x, const Vector& u) const {
    Vector xdot(num_states);
    xdot = Vector::Zero(num_states);

    Scalar phi = x(3);
    Scalar theta = x(4);
    Scalar psi = x(5);
    Scalar phidot = x(9);
    Scalar thetadot = x(10);
    Scalar psidot = x(11);

    Scalar w1 = u(0);
    Scalar w2 = u(1);
    Scalar w3 = u(2);
    Scalar w4 = u(3);

    Eigen::Matrix<Scalar, 3, 1> rpy(phi, theta, psi);
    Eigen::Matrix<Scalar, 3, 3> R = rpy2rotmat(rpy);
    
    Scalar F1 = kf * w1;
    Scalar F2 = kf * w2;
    Scalar F3 = kf * w3;
    Scalar F4 = kf * w4;

    Scalar M1 = km * w1;
    Scalar M2 = km * w2;
    Scalar M3 = km * w3;
    Scalar M4 = km * w4;

    Eigen::Matrix<Scalar, 3, 1> gvec(0, 0, -m * g);
    Eigen::Matrix<Scalar, 3, 1> forcevec(0, 0, F1 + F2 + F3 + F4);
    Eigen::Matrix<Scalar, 3, 1> xyz_ddot = (1.0 / m) * (gvec + R * forcevec);

    Eigen::Matrix<Scalar, 3, 1> rpydot(phidot, thetadot, psidot);
    Eigen::Matrix<Scalar, 3, 1> pqr;
    rpydot2angularvel(rpy, rpydot, pqr);
    pqr = R.adjoint() * pqr;

    Eigen::Matrix<Scalar, 3, 1> pqr_dot_term1;
    pqr_dot_term1 << L * (F2 - F4), L * (F3 - F1), (M1 - M2 + M3 - M4);

    //pqr.cross(I * pqr) won't compile with AutoDiffScalar types?
    auto a = pqr;
    auto b = I * pqr;
    Eigen::Matrix<Scalar, 3, 1> cross(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);

    Eigen::Matrix<Scalar, 3, 1> pqr_dot = I.ldlt().solve(pqr_dot_term1 - cross);
    Eigen::Matrix<Scalar, 3, 3> Phi;
    typename Gradient<Eigen::Matrix<Scalar, 3, 3>, 3>::type dPhi;
    auto ddPhi = (typename Gradient<Eigen::Matrix<Scalar, 3, 3>, 3, 2>::type*) nullptr;
    angularvel2rpydotMatrix(rpy, Phi, &dPhi, ddPhi);

    Eigen::Matrix<Scalar, 9, 3> drpy2drotmat = drpy2rotmat(rpy);
    Eigen::Matrix<Scalar, 9, 1> Rdot_vec;
    Rdot_vec = drpy2drotmat * rpydot;
    Eigen::Matrix<Scalar, 3, 3> Rdot = Eigen::Map< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> >(Rdot_vec.data(), 3, 3);

    Eigen::Matrix<Scalar, 9, 1> dPhi_x_rpydot_vec;
    dPhi_x_rpydot_vec = dPhi * rpydot;
    Eigen::Matrix<Scalar, 3, 3> dPhi_x_rpydot = Eigen::Map< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> >(dPhi_x_rpydot_vec.data(), 3, 3);
    Eigen::Matrix<Scalar, 3, 1> rpy_ddot = Phi * R * pqr_dot + dPhi_x_rpydot * R * pqr + Phi * Rdot * pqr;

    xdot << x.template tail<6>(), xyz_ddot, rpy_ddot;

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
