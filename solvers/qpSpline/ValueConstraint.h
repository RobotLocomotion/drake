#ifndef DRAKE_SOLVERS_QPSPLINE_VALUECONSTRAINT_H_
#define DRAKE_SOLVERS_QPSPLINE_VALUECONSTRAINT_H_

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeSplineGeneration_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

class DLLEXPORT ValueConstraint
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
