#ifndef DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIAL_H_
#define DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIAL_H_

#include <Eigen/Core>
#include "PiecewisePolynomialBase.h"
#include "Polynomial.h"
#include <vector>
#include <random>
#include <limits>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
#if defined(drakeTrajectories_EXPORTS)
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
  typedef Eigen::Ref<CoefficientMatrix> CoefficientMatrixRef;

private:
  std::vector<PolynomialMatrix> polynomials; // a PolynomialMatrix for each piece

public:
  virtual ~PiecewisePolynomial() { };

  // default constructor; just leaves segment_times and polynomials empty
  PiecewisePolynomial();

  // single segment and/or constant value constructor
  template <typename Derived>
  PiecewisePolynomial(const Eigen::MatrixBase<Derived>& value) :
    PiecewisePolynomialBase(std::vector<double>({ { 0.0, std::numeric_limits<double>::infinity() } }))
  {
    polynomials.push_back(value.template cast<PolynomialType>());
  }

  // Matrix constructor
  PiecewisePolynomial(std::vector<PolynomialMatrix> const& polynomials, std::vector<double> const& segment_times);

  // Scalar constructor
  PiecewisePolynomial(std::vector<PolynomialType> const& polynomials, std::vector<double> const& segment_times);

  PiecewisePolynomial derivative(int derivative_order = 1) const;

  PiecewisePolynomial integral(double value_at_start_time = 0.0) const;

  PiecewisePolynomial integral(const CoefficientMatrixRef& value_at_start_time) const;

  double scalarValue(double t, Eigen::DenseIndex row = 0, Eigen::DenseIndex col = 0);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> value(double t) const;

  const PolynomialMatrix& getPolynomialMatrix(int segment_index) const;

  const PolynomialType& getPolynomial(int segment_index, Eigen::DenseIndex row = 0, Eigen::DenseIndex col = 0) const;

  virtual int getSegmentPolynomialDegree(int segment_index, Eigen::DenseIndex row = 0, Eigen::DenseIndex col = 0) const;

  virtual Eigen::DenseIndex rows() const;

  virtual Eigen::DenseIndex cols() const;

  PiecewisePolynomial& operator+=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator-=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator*=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator+=(const CoefficientMatrix& offset);

  PiecewisePolynomial& operator-=(const CoefficientMatrix& offset);

  const PiecewisePolynomial operator+(const PiecewisePolynomial& other) const;

  const PiecewisePolynomial operator-(const PiecewisePolynomial& other) const;

  const PiecewisePolynomial operator*(const PiecewisePolynomial& other) const;

  const PiecewisePolynomial operator+(const CoefficientMatrix& offset) const;

  const PiecewisePolynomial operator-(const CoefficientMatrix& offset) const;

  bool isApprox(const PiecewisePolynomial& other, double tol) const;

  void shiftRight(double offset);

  void setPolynomialMatrixBlock(const PolynomialMatrix& replacement, int segment_number, Eigen::DenseIndex row_start = 0, Eigen::DenseIndex col_start = 0);

  PiecewisePolynomial slice(int start_segment_index, int num_segments);

  static PiecewisePolynomial random(
    Eigen::DenseIndex rows, Eigen::DenseIndex cols, Eigen::DenseIndex num_coefficients_per_polynomial,
    const std::vector<double>& segment_times);

protected:
  double segmentValueAtGlobalAbscissa(int segment_index, double t, Eigen::DenseIndex row, Eigen::DenseIndex col) const;
};

#endif /* DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIAL_H_ */
