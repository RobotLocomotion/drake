#include "PiecewisePolynomial.h"
#include <stdexcept>
#include <cassert>
#include <algorithm>
#include <iostream>

using namespace std;

PiecewisePolynomial::PiecewisePolynomial(std::vector<Polynomial> const& polynomials, std::vector<double> const& segment_times) :
    PiecewisePolynomialBase(segment_times),
    polynomials(polynomials)
{
  assert(segment_times.size() == (polynomials.size() + 1));

  for (int i = 0; i < polynomials.size(); i++) {
    if (segment_times[i + 1] < segment_times[i])
      throw runtime_error("times must be increasing");
  }
}

int PiecewisePolynomial::getSegmentIndex(double t) {
  // clip to min/max times
  if (t < getStartTime())
    t = getStartTime();

  if (t > getEndTime())
    t = getEndTime();

  int segment_index = 0;
  while (t >= getEndTime(segment_index) && segment_index < getNumberOfSegments())
    segment_index++;

  return segment_index;
}

double PiecewisePolynomial::derivativeValue(int derivative_order, double t) {
  return polynomials[getSegmentIndex(t)].derivativeValue(derivative_order, t);
}

int PiecewisePolynomial::getSegmentPolynomialOrder(int segment_number) const {
  return polynomials[segment_number].getOrder();
}
