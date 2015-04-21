#include "PiecewisePolynomial.h"
#include <stdexcept>
#include <cassert>
#include <algorithm>
#include <complex>

using namespace std;

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>::PiecewisePolynomial(std::vector<Polynomial<CoefficientType>> const& polynomials, std::vector<double> const& segment_times) :
    PiecewisePolynomialBase(segment_times),
    polynomials(polynomials)
{
  assert(segment_times.size() == (polynomials.size() + 1));

  for (int i = 0; i < polynomials.size(); i++) {
    if (segment_times[i + 1] < segment_times[i])
      throw runtime_error("times must be increasing");
  }
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>::PiecewisePolynomial() {
  // empty
}

template <typename CoefficientType>
int PiecewisePolynomial<CoefficientType>::getSegmentIndex(double t) {
  // clip to min/max times
  if (t < getStartTime())
    t = getStartTime();

  if (t > getEndTime())
    t = getEndTime();

  int segment_index = 0;
  while (t >= getEndTime(segment_index) && segment_index < getNumberOfSegments() - 1)
    segment_index++;

  return segment_index;
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType> PiecewisePolynomial<CoefficientType>::derivative(int derivative_order) const {
  vector<Polynomial<CoefficientType>> derivative_polynomials;
  derivative_polynomials.reserve(polynomials.size());
  for (auto it = polynomials.begin(); it != polynomials.end(); ++it) {
    derivative_polynomials.push_back(it->derivative(derivative_order));
  }
  return PiecewisePolynomial<CoefficientType>(derivative_polynomials, segment_times);
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType> PiecewisePolynomial<CoefficientType>::integral(double value_at_start_time) const {
  PiecewisePolynomial<CoefficientType> ret;
  ret.segment_times = segment_times;
  ret.polynomials.reserve(polynomials.size());
  ret.polynomials.push_back(polynomials[0].integral(value_at_start_time));
  for (int i = 1; i < getNumberOfSegments(); i++) {
    ret.polynomials.push_back(polynomials[i].integral(ret.segmentValueAtGlobalAbscissa(i - 1, getStartTime(i))));
  }
  return ret;
}

template <typename CoefficientType>
double PiecewisePolynomial<CoefficientType>::value(double t) {
  int segment_index = getSegmentIndex(t);
  return segmentValueAtGlobalAbscissa(segment_index, t);
}

template <typename CoefficientType>
const Polynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::getPolynomial(int segment_index) const {
  segmentNumberRangeCheck(segment_index);
  return polynomials[segment_index];
}

template <typename CoefficientType>
int PiecewisePolynomial<CoefficientType>::getSegmentPolynomialDegree(int segment_index) const {
  segmentNumberRangeCheck(segment_index);
  return polynomials[segment_index].getDegree();
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::operator+=(const PiecewisePolynomial<CoefficientType>& other) {
  if (!segmentTimesEqual(other, 1e-10))
    throw runtime_error("Addition not yet implemented when segment times are not equal");
  for (int i = 0; i < polynomials.size(); i++)
    polynomials[i] += other.polynomials[i];
  return *this;
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::operator*=(const PiecewisePolynomial<CoefficientType>& other) {
  if (!segmentTimesEqual(other, 1e-10))
    throw runtime_error("Multiplication not yet implemented when segment times are not equal");
  for (int i = 0; i < polynomials.size(); i++)
    polynomials[i] *= other.polynomials[i];
  return *this;
}

template <typename CoefficientType>
const PiecewisePolynomial<CoefficientType> PiecewisePolynomial<CoefficientType>::operator+(const PiecewisePolynomial<CoefficientType> &other) const {
  PiecewisePolynomial<CoefficientType> ret = *this;
  ret += other;
  return ret;
}

template <typename CoefficientType>
const PiecewisePolynomial<CoefficientType> PiecewisePolynomial<CoefficientType>::operator*(const PiecewisePolynomial<CoefficientType> &other) const {
  PiecewisePolynomial<CoefficientType> ret = *this;
  ret *= other;
  return ret;
}

template <typename CoefficientType>
bool PiecewisePolynomial<CoefficientType>::isApprox(const PiecewisePolynomial<CoefficientType>& other, double tol) const {
  if (!segmentTimesEqual(other, tol))
    return false;

  for (int i = 0; i < getNumberOfSegments(); ++i) {
    if (!polynomials[i].isApprox(other.polynomials[i], tol))
      return false;
  }
  return true;
}

template <typename CoefficientType>
double PiecewisePolynomial<CoefficientType>::segmentValueAtGlobalAbscissa(int segment_index, double t) const {
  return polynomials[segment_index].value(t - getStartTime(segment_index));
}

template class DLLEXPORT PiecewisePolynomial<double>;
//template class DLLEXPORT PiecewisePolynomial<std::complex<double>>; // doesn't work yet
