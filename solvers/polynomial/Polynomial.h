#ifndef DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_
#define DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_

#include <Eigen/Core>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakePolynomial_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

class DLLEXPORT Polynomial
{
private:
  Eigen::VectorXd coefficients;

public:
  Polynomial(Eigen::Ref<Eigen::VectorXd> const& coefficients);

  Polynomial(int num_coefficients);

  int getNumberOfCoefficients() const;

  int getOrder() const;

  Eigen::VectorXd const& getCoefficients() const;

  double value(double t) const;

  Polynomial derivative(int derivative_order = 1) const;

  Polynomial integral(double integration_constant = 0.0) const;

  Polynomial& operator+=(const Polynomial& other);

  Polynomial& operator*=(const Polynomial& other);

  const Polynomial operator+(const Polynomial &other) const;

  const Polynomial operator*(const Polynomial &other) const;

  bool isApprox(const Polynomial& other, double tol) const;

  static Polynomial zero();

private:
  double valueHorner(double t) const;

  double valueStabilizedHorner(double t) const;
};

#endif /* DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_ */
