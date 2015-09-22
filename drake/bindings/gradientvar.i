%fragment("gradientvar_typemaps", "header")
%{
  #include<iostream>
  template<typename Scalar, int Rows, int Cols>
  Eigen::MatrixXd nthGradientValue(const GradientVar<Scalar, Rows, Cols> &gvar, int n) {
    if (n <= 0) {
      Eigen::MatrixXd value;
      value = gvar.value();
      return value;
    } else {
      return nthGradientValue(gvar.gradient(), n-1);
    }
  }
%}

#ifdef SWIGPYTHON
%define %gradientvar_typemaps(SCALAR, ROWS, COLS)

%typemap(out, fragment="gradientvar_typemaps") GradientVar< SCALAR, ROWS, COLS >
{
  int num_elements = $1.maxOrder() + 1;
  $result = PyTuple_New(num_elements);

  for (int i=0; i < num_elements; i++) {
    MatrixXd gval;
    try {
      gval = nthGradientValue(*(&($1)), i);
    } catch (const std::exception& e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
    }
    PyObject* pyarray;
    if (!ConvertFromEigenToNumPyMatrix< Eigen::MatrixXd >(&pyarray, &gval))
      SWIG_fail;
    PyTuple_SetItem($result, i, pyarray);
  }
}
%enddef
#endif

#ifdef SWIGMATLAB
%define %gradientvar_typemaps(SCALAR, ROWS, COLS)

%typemap(out, fragment="gradientvar_typemaps") GradientVar< SCALAR, ROWS, COLS >
{
  int num_elements = $1.maxOrder() + 1;
  $result = mxCreateCellMatrix(1, num_elements);

  for (int i=0; i < num_elements; i++) {
    MatrixXd gval = nthGradientValue(*(&($1)), i);
    mxArray* pobj;
    if (!ConvertFromEigenToMatlabMatrix< Eigen::MatrixXd >(&pobj, &gval))
      SWIG_fail;
    mxSetCell($result, i, pobj);
  }
}
%enddef
#endif

%gradientvar_typemaps(Eigen::VectorXd::Scalar, Eigen::Dynamic, Eigen::Dynamic)
%gradientvar_typemaps(Eigen::VectorXd::Scalar, Eigen::Dynamic, 1)
%gradientvar_typemaps(Eigen::VectorXd::Scalar, SPACE_DIMENSION, 1)
%gradientvar_typemaps(Eigen::VectorXd::Scalar,Eigen::Dynamic,Eigen::VectorXd::ColsAtCompileTime)
%import "GradientVar.h"