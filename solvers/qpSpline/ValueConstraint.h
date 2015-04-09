#ifndef DRAKE_SOLVERS_QPSPLINE_VALUECONSTRAINT_H_
#define DRAKE_SOLVERS_QPSPLINE_VALUECONSTRAINT_H_

class ValueConstraint
{
private:
  int derivative_order;
  double time;
  double value;

public:
  ValueConstraint(int derivative_order, double time, double value);
  int getDerivativeOrder() const;
  double getTime() const;
  double getValue() const;
};

#endif /* DRAKE_SOLVERS_QPSPLINE_VALUECONSTRAINT_H_ */
