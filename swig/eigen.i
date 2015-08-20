/*
 * The Biomechanical ToolKit
 * Copyright (c) 2009-2013, Arnaud Barré
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name(s) of the copyright holders nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 /*
 Changes:
 Robin Deits:
  * modified the typemaps to throw SWIG_fail when errors occur, which results in helpful Python errors being thrown when the type conversions fail
  * added typemaps for std::vector containers
  * updated to use the new numpy API for full python3 support
 */

%{
  #define SWIG_FILE_WITH_INIT
  #include "Eigen/Core"
  #include <vector>
%}

%include "numpy.i"

%init
%{
  import_array();
%}

%fragment("Eigen_Fragments", "header",  fragment="NumPy_Fragments")
%{
  template <typename T> int NumPyType() {return -1;};

  template <class Derived>
  bool ConvertFromNumpyToEigenMatrix(Eigen::MatrixBase<Derived>* out, PyObject* in)
  {
    int rows = 0;
    int cols = 0;
    // Check object type
    if (!is_array(in))
    {
      PyErr_SetString(PyExc_ValueError, "The given input is not known as a NumPy array or matrix.");
      return false;
    }
    // Check data type
    else if (array_type(in) != NumPyType<typename Derived::Scalar>())
    {
      PyErr_SetString(PyExc_ValueError, "Type mismatch between NumPy and Eigen objects.");
      return false;
    }
    // Check dimensions
    else if (array_numdims(in) > 2)
    {
      PyErr_SetString(PyExc_ValueError, "Eigen only support 1D or 2D array.");
      return false;
    }
    else if (array_numdims(in) == 1)
    {
      rows = array_size(in,0);
      cols = 1;
      if ((Derived::RowsAtCompileTime != Eigen::Dynamic) && (Derived::RowsAtCompileTime != rows))
      {
        PyErr_SetString(PyExc_ValueError, "Row dimension mismatch between NumPy and Eigen objects (1D).");
        return false;
      }
      else if ((Derived::ColsAtCompileTime != Eigen::Dynamic) && (Derived::ColsAtCompileTime != 1))
      {
        PyErr_SetString(PyExc_ValueError, "Column dimension mismatch between NumPy and Eigen objects (1D).");
        return false;
      }
    }
    else if (array_numdims(in) == 2)
    {
      rows = array_size(in,0);
      cols = array_size(in,1);
      if ((Derived::RowsAtCompileTime != Eigen::Dynamic) && (Derived::RowsAtCompileTime != array_size(in,0)))
      {
        PyErr_SetString(PyExc_ValueError, "Row dimension mismatch between NumPy and Eigen objects (2D).");
        return false;
      }
      else if ((Derived::ColsAtCompileTime != Eigen::Dynamic) && (Derived::ColsAtCompileTime != array_size(in,1)))
      {
        PyErr_SetString(PyExc_ValueError, "Column dimension mismatch between NumPy and Eigen objects (2D).");
        return false;
      }
    }
    // Extract data
    int isNewObject = 0;
    PyArrayObject* temp = obj_to_array_contiguous_allow_conversion(in, array_type(in), &isNewObject);
    if (temp == NULL)
    {
      PyErr_SetString(PyExc_ValueError, "Impossible to convert the input into a Python array object.");
      return false;
    }
    out->derived().setZero(rows, cols);
    typename Derived::Scalar* data = static_cast<typename Derived::Scalar*>(PyArray_DATA(temp));
    for (int i = 0; i != rows; ++i)
      for (int j = 0; j != cols; ++j)
        out->coeffRef(i,j) = data[i*cols+j];

    return true;
  };

  // Copies values from Eigen type into an existing NumPy type
  template <class Derived>
  bool CopyFromEigenToNumPyMatrix(PyObject* out, Eigen::MatrixBase<Derived>* in)
  {
    int rows = 0;
    int cols = 0;
    // Check object type
    if (!is_array(out))
    {
      PyErr_SetString(PyExc_ValueError, "The given input is not known as a NumPy array or matrix.");
      return false;
    }
    // Check data type
    else if (array_type(out) != NumPyType<typename Derived::Scalar>())
    {
      PyErr_SetString(PyExc_ValueError, "Type mismatch between NumPy and Eigen objects.");
      return false;
    }
    // Check dimensions
    else if (array_numdims(out) > 2)
    {
      PyErr_SetString(PyExc_ValueError, "Eigen only support 1D or 2D array.");
      return false;
    }
    else if (array_numdims(out) == 1)
    {
      rows = array_size(out,0);
      cols = 1;
      if ((Derived::RowsAtCompileTime != Eigen::Dynamic) && (Derived::RowsAtCompileTime != rows))
      {
        PyErr_SetString(PyExc_ValueError, "Row dimension mismatch between NumPy and Eigen objects (1D).");
        return false;
      }
      else if ((Derived::ColsAtCompileTime != Eigen::Dynamic) && (Derived::ColsAtCompileTime != 1))
      {
        PyErr_SetString(PyExc_ValueError, "Column dimension mismatch between NumPy and Eigen objects (1D).");
        return false;
      }
    }
    else if (array_numdims(out) == 2)
    {
      rows = array_size(out,0);
      cols = array_size(out,1);
    }

    if (in->cols() != cols || in->rows() != rows) {
      /// TODO: be forgiving and simply create or resize the array
      PyErr_SetString(PyExc_ValueError, "Dimension mismatch between NumPy and Eigen object (return argument).");
      return false;
    }

    // Extract data
    int isNewObject = 0;
    PyArrayObject* temp = obj_to_array_contiguous_allow_conversion(out, array_type(out), &isNewObject);
    if (temp == NULL)
    {
      PyErr_SetString(PyExc_ValueError, "Impossible to convert the input into a Python array object.");
      return false;
    }

    typename Derived::Scalar* data = static_cast<typename Derived::Scalar*>(PyArray_DATA(temp));

    for (int i = 0; i != in->rows(); ++i) {
      for (int j = 0; j != in->cols(); ++j) {
        data[i*in->cols()+j] = in->coeff(i,j);
      }
    }

    return true;
  };

  template <class Derived>
  bool ConvertFromEigenToNumPyMatrix(PyObject** out, Eigen::MatrixBase<Derived>* in)
  {
    npy_intp dims[2] = {in->rows(), in->cols()};
    *out = PyArray_SimpleNew(2, dims, NumPyType<typename Derived::Scalar>());
    if (!out)
      return false;
    typename Derived::Scalar* data = static_cast<typename Derived::Scalar*>(PyArray_DATA((PyArrayObject*) *out));
    for (int i = 0; i != dims[0]; ++i)
      for (int j = 0; j != dims[1]; ++j)
        data[i*dims[1]+j] = in->coeff(i,j);
    return true;
  };

  template<> int NumPyType<double>() {return NPY_DOUBLE;};
%}

// ----------------------------------------------------------------------------
// Macro to create the typemap for Eigen classes
// ----------------------------------------------------------------------------
%define %eigen_typemaps(CLASS...)

// Argout: const & (Disabled and prevents calling of the non-const typemap)
%typemap(argout, fragment="Eigen_Fragments") const CLASS & ""

// Argout: & (for returning values to in-out arguments)
%typemap(argout, fragment="Eigen_Fragments") CLASS &
{
  // Argout: &
  if (!CopyFromEigenToNumPyMatrix<CLASS >($input, $1))
    SWIG_fail;
}

// In: (nothing: no constness)
%typemap(in, fragment="Eigen_Fragments") CLASS (CLASS temp)
{
  if (!ConvertFromNumpyToEigenMatrix<CLASS >(&temp, $input))
    SWIG_fail;
  $1 = temp;
}
// In: const&
%typemap(in, fragment="Eigen_Fragments") CLASS const& (CLASS temp)
{
  // In: const&
  if (!ConvertFromNumpyToEigenMatrix<CLASS >(&temp, $input))
    SWIG_fail;
  $1 = &temp;
}
// In: & (not yet implemented)
%typemap(in, fragment="Eigen_Fragments") CLASS & (CLASS temp)
{
  // In: non-const&
  if (!ConvertFromNumpyToEigenMatrix<CLASS >(&temp, $input))
    SWIG_fail;

  $1 = &temp;
}
// In: const* (not yet implemented)
%typemap(in, fragment="Eigen_Fragments") CLASS const*
{
  PyErr_SetString(PyExc_ValueError, "The input typemap for const pointer is not yet implemented. Please report this problem to the developer.");
  SWIG_fail;
}
// In: * (not yet implemented)
%typemap(in, fragment="Eigen_Fragments") CLASS *
{
  PyErr_SetString(PyExc_ValueError, "The input typemap for non-const pointer is not yet implemented. Please report this problem to the developer.");
  SWIG_fail;
}

// Out: (nothing: no constness)
%typemap(out, fragment="Eigen_Fragments") CLASS
{
  if (!ConvertFromEigenToNumPyMatrix<CLASS >(&$result, &$1))
    SWIG_fail;
}
// Out: const
%typemap(out, fragment="Eigen_Fragments") CLASS const
{
  if (!ConvertFromEigenToNumPyMatrix<CLASS >(&$result, &$1))
    SWIG_fail;
}
// Out: const&
%typemap(out, fragment="Eigen_Fragments") CLASS const&
{
  if (!ConvertFromEigenToNumPyMatrix<CLASS >(&$result, $1))
    SWIG_fail;
}
// Out: & (not yet implemented)
%typemap(out, fragment="Eigen_Fragments") CLASS &
{
  PyErr_SetString(PyExc_ValueError, "The output typemap for non-const reference is not yet implemented. Please report this problem to the developer.");
  SWIG_fail;
}
// Out: const* (not yet implemented)
%typemap(out, fragment="Eigen_Fragments") CLASS const*
{
  PyErr_SetString(PyExc_ValueError, "The output typemap for const pointer is not yet implemented. Please report this problem to the developer.");
  SWIG_fail;
}
// Out: * (not yet implemented)
%typemap(out, fragment="Eigen_Fragments") CLASS *
{
  PyErr_SetString(PyExc_ValueError, "The output typemap for non-const pointer is not yet implemented. Please report this problem to the developer.");
  SWIG_fail;
}

%typemap(out, fragment="Eigen_Fragments") std::vector<CLASS >
{
  $result = PyList_New($1.size());
  if (!$result)
    SWIG_fail;
  for (size_t i=0; i != $1.size(); ++i) {
    PyObject *out;
    if (!ConvertFromEigenToNumPyMatrix(&out, &$1[i]))
      SWIG_fail;
    if (PyList_SetItem($result, i, out) == -1)
      SWIG_fail;
  }
}

%typemap(in, fragment="Eigen_Fragments") std::vector<CLASS >
{
  if (!PyList_Check($input))
    SWIG_fail;
  $1.resize(PyList_Size($input));
  for (size_t i=0; i != PyList_Size($input); ++i) {
    if (!ConvertFromNumpyToEigenMatrix<CLASS >(&$1[i], PyList_GetItem($input, i)))
      SWIG_fail;
  }
}

// %typemap(in, fragment="Eigen_Fragments") const Eigen::Ref<CLASS > & (CLASS temp)
// {
//   if (!ConvertFromNumpyToEigenMatrix<CLASS >(&temp, $input))
//     SWIG_fail;
//   new ($1) Eigen::Ref<CLASS >(temp);
//   // $1->construct(temp);
// }
%enddef
