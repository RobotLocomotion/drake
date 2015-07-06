#ifndef DRAKE_SOLVERS_QPSPLINE_CONTINUITYCONSTRAINT_H_
#define DRAKE_SOLVERS_QPSPLINE_CONTINUITYCONSTRAINT_H_

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

class DLLEXPORT ContinuityConstraint
{
private:
  int derivative_order;
  int first_spline_index;
  int second_spline_index;

public:
  ContinuityConstraint(int first_spline_index, int second_spline_index, int derivative_order);
  int getDerivativeOrder() const;
  int getFirstSplineIndex() const;
  int getSecondSplineIndex() const;
};

#endif /* DRAKE_SOLVERS_QPSPLINE_CONTINUITYCONSTRAINT_H_ */
