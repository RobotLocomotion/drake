#ifndef DRAKE_LINEARSYSTEM_H
#define DRAKE_LINEARSYSTEM_H

#include "DrakeSystem.h"

/// Implements
///   xcdot = Ac*x + Bc*u + xcdot0
///   xdn = Ad*x + Bd*u + xdn0
///   y = C*x + D*u + y0

class AffineSystem : public DrakeSystem {
public:
  AffineSystem(const std::string& name,
               const Eigen::MatrixXd& _Ac,const Eigen::MatrixXd& _Bc,const Eigen::VectorXd& _xcdot0,
               const Eigen::MatrixXd& _Ad,const Eigen::MatrixXd& _Bd,const Eigen::VectorXd& _xdn0,
               const Eigen::MatrixXd& _C,const Eigen::MatrixXd& _D,const Eigen::VectorXd& _y0);

  virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return Ac*x + Bc*u + xcdot0;
  }
  virtual Eigen::VectorXd update(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return Ad*x + Bd*u + xdn0;
  }
  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    return C*x + D*u + y0;
  }

  virtual bool isTimeInvariant() const override { return true; }
  virtual bool isDirectFeedthrough() const override { return D.isZero(); }

private:
  Eigen::MatrixXd Ac, Bc, Ad, Bd, C, D;
  Eigen::VectorXd xcdot0, xdn0, y0;
};

class LinearSystem : public AffineSystem {
public:
  LinearSystem(const std::string& name,
               const Eigen::MatrixXd& _Ac,const Eigen::MatrixXd& _Bc,
               const Eigen::MatrixXd& _Ad,const Eigen::MatrixXd& _Bd,
               const Eigen::MatrixXd& _C,const Eigen::MatrixXd& _D);
};

#endif //DRAKE_LINEARSYSTEM_H
