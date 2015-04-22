#ifndef DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIAL_H_
#define DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIAL_H_

#include <Eigen/Core>
#include "PiecewisePolynomialBase.h"
#include "Polynomial.h"
#include <vector>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
#if defined(drakePiecewisePolynomial_EXPORTS)
#define DLLEXPORT __declspec( dllexport )
#else
#define DLLEXPORT __declspec( dllimport )
#endif
#else
#define DLLEXPORT
#endif

template<typename CoefficientType = double>
class DLLEXPORT PiecewisePolynomial: public PiecewisePolynomialBase
{
public:
  typedef Polynomial<CoefficientType> PolynomialType;
  typedef Eigen::Matrix<PolynomialType, Eigen::Dynamic, Eigen::Dynamic> PolynomialMatrix;
  typedef Eigen::Matrix<CoefficientType, Eigen::Dynamic, Eigen::Dynamic> CoefficientMatrix;

private:
  std::vector<PolynomialMatrix> polynomials; // a PolynomialMatrix for each piece

  int getSegmentIndex(double t);

public:
  virtual ~PiecewisePolynomial() { };

  // Matrix constructor
  PiecewisePolynomial(std::vector<PolynomialMatrix> const& polynomials, std::vector<double> const& segment_times);

  // Scalar constructor
  PiecewisePolynomial(std::vector<PolynomialType> const& polynomials, std::vector<double> const& segment_times);

  PiecewisePolynomial derivative(int derivative_order = 1) const;

  PiecewisePolynomial integral(double value_at_start_time = 0.0) const;

  PiecewisePolynomial integral(const Eigen::Ref<CoefficientMatrix>& value_at_start_time) const;

  double value(double t, Eigen::DenseIndex row = 0, Eigen::DenseIndex col = 0);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrixValue(double t);

  const PolynomialMatrix& getPolynomialMatrix(int segment_index) const;

  const PolynomialType& getPolynomial(int segment_index, Eigen::DenseIndex row = 0, Eigen::DenseIndex col = 0) const;

  virtual int getSegmentPolynomialDegree(int segment_index, Eigen::DenseIndex row = 0, Eigen::DenseIndex col = 0) const;

  virtual Eigen::DenseIndex rows() const;

  virtual Eigen::DenseIndex cols() const;

  PiecewisePolynomial& operator+=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator*=(const PiecewisePolynomial& other);

  const PiecewisePolynomial operator+(const PiecewisePolynomial &other) const;

  const PiecewisePolynomial operator*(const PiecewisePolynomial &other) const;

  bool isApprox(const PiecewisePolynomial& other, double tol) const;

protected:
  PiecewisePolynomial();

  double segmentValueAtGlobalAbscissa(int segment_index, double t, Eigen::DenseIndex row, Eigen::DenseIndex col) const;
};

#endif /* DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIAL_H_ */
