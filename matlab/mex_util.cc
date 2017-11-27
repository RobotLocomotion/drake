#include "mex_util.h"

#include <limits>

// mxIsClass seems to not be able to handle derived classes, so I will
// implement what I need by calling back to MATLAB.
bool isa(const mxArray* mxa, const char* class_str) {
  mxArray* plhs;
  mxArray* prhs[2];
  prhs[0] = const_cast<mxArray*>(mxa);
  prhs[1] = mxCreateString(class_str);
  mexCallMATLAB(1, &plhs, 2, prhs, "isa");
  bool tf = *mxGetLogicals(plhs);
  mxDestroyArray(plhs);
  mxDestroyArray(prhs[1]);
  return tf;
}

bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[],
                       const char* filename) {
  mxArray* ex = mexCallMATLABWithTrap(nlhs, plhs, nrhs, prhs, filename);

  if (ex != nullptr) {
    mexPrintf(
        "DrakeSystem mexCallMATLABsafe: error when calling ''%s'' with the "
        "following "
        "arguments:\n",
        filename);

    for (int i = 0; i < nrhs; i++) {
      mexCallMATLAB(0, nullptr, 1, &prhs[i], "disp");
    }

    mxArray* report;
    mexCallMATLAB(1, &report, 1, &ex, "getReport");
    char* errmsg = mxArrayToString(report);
    mexPrintf(errmsg);
    mxFree(errmsg);
    mxDestroyArray(report);
    mxDestroyArray(ex);
    return true;
  }

  for (int i = 0; i < nlhs; i++) {
    if (plhs[i] == nullptr) {
      mexPrintf(
          "Drake mexCallMATLABsafe: error when calling ''%s'' with the "
          "following arguments:\n",
          filename);

      for (i = 0; i < nrhs; i++) {
        mexCallMATLAB(0, nullptr, 1, &prhs[i], "disp");
      }

      mexPrintf(
          "Not Enough Outputs: Asked for %d outputs, but function only "
          "returned %d.\n",
          nrhs, i);
      return true;
    }
  }
  return false;
}

/*
 * @param subclass_name (optional) if you want to call a class that derives
 * from DrakeMexPointer (e.g., so that you can refer to it as something more
 * specific in your MATLAB code), then you can pass in the alternative name
 * here. The constructor for this class must take the same inputs as the
 * DrakeMexPointer constructor.
 */
mxArray* createDrakeMexPointer(void* ptr, const std::string& name, int type_id,
                               int num_additional_inputs,
                               mxArray* delete_fcn_additional_inputs[],
                               const std::string& subclass_name,
                               const std::string& mex_function_name_prefix) {
  mxClassID cid;

  if (sizeof(ptr) == 4) {
    cid = mxUINT32_CLASS;
  } else if (sizeof(ptr) == 8) {
    cid = mxUINT64_CLASS;
  } else {
    mexErrMsgIdAndTxt("Drake:constructDrakeMexPointer:PointerSize",
                      "Are you on a 32-bit machine or 64-bit machine??");
  }

  const int nrhs = 4 + num_additional_inputs;
  mxArray* plhs[1];
  mxArray** prhs;
  prhs = new mxArray*[nrhs];

  prhs[0] = mxCreateNumericMatrix(1, 1, cid, mxREAL);
  std::memcpy(mxGetData(prhs[0]), &ptr, sizeof(ptr));

  prhs[1] =
      mxCreateString((mex_function_name_prefix + mexFunctionName()).c_str());

  prhs[2] = mxCreateString(name.c_str());

  prhs[3] = mxCreateNumericMatrix(1, 1, cid, mxREAL);
  std::memcpy(mxGetData(prhs[3]), &type_id, sizeof(type_id));

  for (int i = 0; i < num_additional_inputs; i++) {
    prhs[4 + i] = delete_fcn_additional_inputs[i];
  }

  // Call MATLAB to construct MEX pointer object.
  if (!subclass_name.empty()) {
    mexCallMATLABsafe(1, plhs, nrhs, prhs, subclass_name.c_str());

    if (!isa(plhs[0], "DrakeMexPointer")) {
      mxDestroyArray(plhs[0]);
      mexErrMsgIdAndTxt(
          "Drake:createDrakeMexPointer:InvalidSubclass",
          "subclass_name is not a valid subclass of DrakeMexPointer.");
    }
  } else {
    mexCallMATLABsafe(1, plhs, nrhs, prhs, "DrakeMexPointer");
  }

  mexLock();

  delete[] prhs;
  return plhs[0];
}

void* getDrakeMexPointer(const mxArray* mx) {
  if (mx == nullptr) {
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs", "null mxArray.");
  }

  void* ptr = nullptr;

  // TODO(russt): Optimize this by caching the pointer values, as described in
  // http://groups.csail.mit.edu/locomotion/bugs/show_bug.cgi?id=1590.
  mxArray* ptr_array = mxGetProperty(mx, 0, "ptr");

  if (ptr_array == nullptr) {
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs",
                      "Cannot retrieve 'ptr' field from this mxArray. Are you "
                      "sure it is a valid DrakeMexPointer object?");
  }

  switch (sizeof(void*)) {
    case 4:
      if (!mxIsUint32(ptr_array)) {
        mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadPointerSize",
                          "DrakeMexPointer expected a 32-bit ptr field but got "
                          "something else.");
      }
      break;
    case 8:
      if (!mxIsUint64(ptr_array)) {
        mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadPointerSize",
                          "DrakeMexPointer expected a 64-bit ptr field but got "
                          "something else.");
      }
      break;
    default:
      mexErrMsgIdAndTxt(
          "Drake:getDrakeMexPointer:BadPointerSize",
          "DrakeMexPointer got a pointer that was neither 32-bit nor 64-bit.");
  }

  if (!mxIsNumeric(ptr_array) || mxGetNumberOfElements(ptr_array) != 1) {
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs",
                      "The ptr property of this DrakeMexPointer does not "
                      "appear to contain a valid pointer.");
  }

  std::memcpy(&ptr, mxGetData(ptr_array),
              sizeof(ptr));  // Could use a reinterpret_cast here instead.

  return ptr;
}

Eigen::SparseMatrix<double> matlabToEigenSparse(const mxArray* mex) {
  auto ir = mxGetIr(mex);
  auto jc = mxGetJc(mex);
  auto pr = mxGetPr(mex);

  auto rows = mxGetM(mex);
  auto cols = mxGetN(mex);
  //  auto num_non_zero_max = mxGetNzmax(mex);
  auto num_non_zero = jc[cols];  // From mxgetnzmax.c example.

  Eigen::SparseMatrix<double> ret(rows, cols);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(num_non_zero);

  for (mwSize col = 0; col < cols; col++) {
    const auto& val_index_start = jc[col];
    const auto& val_index_end = jc[col + 1];

    for (auto val_index = val_index_start; val_index < val_index_end;
         val_index++) {
      const double& val = pr[val_index];
      const auto& row = ir[val_index];
      triplets.emplace_back(row, col, val);
    }
  }

  ret.setFromTriplets(triplets.begin(), triplets.end());
  return ret;
}

std::string mxGetStdString(const mxArray* array) {
  mwSize buffer_length = mxGetNumberOfElements(array) + 1;
  auto* buffer = new char[buffer_length];
  const int status = mxGetString(array, buffer, buffer_length);

  if (status != 0) {
    delete[] buffer;
    throw std::runtime_error(
        "mxGetStdString failed. Possible cause: mxArray is not a string "
        "array.");
  } else {
    std::string ret(buffer);
    delete[] buffer;
    return ret;
  }
}

std::vector<std::string> mxGetVectorOfStdStrings(const mxArray* array) {
  if (!mxIsCell(array)) {
    throw std::runtime_error("The input is not a cell array.");
  }

  std::vector<std::string> strings;
  size_t numel = mxGetNumberOfElements(array);

  for (size_t i = 0; i < numel; i++) {
    strings.push_back(mxGetStdString(mxGetCell(array, i)));
  }

  return strings;
}

mxArray* stdStringToMatlab(const std::string& str) {
  return mxCreateString(str.c_str());
}

mxArray* vectorOfStdStringsToMatlab(const std::vector<std::string>& strs) {
  mxArray* cell = mxCreateCellMatrix(strs.size(), 1);

  for (size_t i = 0; i < strs.size(); i++) {
    mxSetCell(cell, i, mxCreateString(strs[i].c_str()));
  }

  return cell;
}

double* mxGetPrSafe(const mxArray* pobj) {
  if (!mxIsDouble(pobj)) {
    mexErrMsgIdAndTxt("Drake:mxGetPrSafe:BadInputs",
                      "mxGetPr can only be called on arguments which "
                      "correspond to MATLAB doubles.");
  }

  return mxGetPr(pobj);
}

mxArray* mxGetPropertySafe(const mxArray* array,
                           std::string const& field_name) {
  return mxGetPropertySafe(array, 0, field_name);
}

mxArray* mxGetPropertySafe(const mxArray* array, size_t index,
                           std::string const& field_name) {
  mxArray* ret = mxGetProperty(array, index, field_name.c_str());

  if (ret == nullptr) {
    mexErrMsgIdAndTxt("Drake:mxGetPropertySafe",
                      ("Property not found: " + field_name).c_str());
  }

  return ret;
}

mxArray* mxGetFieldSafe(const mxArray* array, std::string const& field_name) {
  return mxGetFieldSafe(array, 0, field_name);
}

mxArray* mxGetFieldSafe(const mxArray* array, size_t index,
                        std::string const& field_name) {
  mxArray* ret = mxGetField(array, index, field_name.c_str());

  if (ret == nullptr) {
    mexErrMsgIdAndTxt("Drake:mxGetFieldSafe",
                      ("Field not found: " + field_name).c_str());
  }

  return ret;
}

void mxSetFieldSafe(mxArray* array, size_t index, std::string const& fieldname,
                    mxArray* data) {
  int fieldnum = mxGetFieldNumber(array, fieldname.c_str());

  if (fieldnum < 0) {
    fieldnum = mxAddField(array, fieldname.c_str());
  }

  mxSetFieldByNumber(array, index, fieldnum, data);
}

mxArray* mxGetFieldOrPropertySafe(const mxArray* array,
                                  std::string const& field_name) {
  return mxGetFieldOrPropertySafe(array, 0, field_name);
}

mxArray* mxGetFieldOrPropertySafe(const mxArray* array, size_t index,
                                  std::string const& field_name) {
  mxArray* field_or_property = mxGetField(array, index, field_name.c_str());

  if (field_or_property == nullptr) {
    field_or_property = mxGetPropertySafe(array, index, field_name);
  }

  return field_or_property;
}

template <typename T>
const std::vector<T> matlabToStdVector(const mxArray* in) {
  // Works for both row vectors and column vectors.

  if (mxGetNumberOfElements(in) == 0) {
    // If input is empty, output is an empty vector.
    return std::vector<T>();
  }

  if (mxGetM(in) != 1 && mxGetN(in) != 1) {
    throw std::runtime_error("Not a vector.");
  }

  std::vector<T> ret;

  if (mxIsLogical(in)) {
    mxLogical* data = mxGetLogicals(in);

    for (size_t i = 0; i < mxGetNumberOfElements(in); i++) {
      ret.push_back(static_cast<T>(data[i]));
    }
  } else {
    double* data = mxGetPrSafe(in);

    for (size_t i = 0; i < mxGetNumberOfElements(in); i++) {
      ret.push_back(static_cast<T>(data[i]));
    }
  }

  return ret;
}

template <>
DLL_EXPORT_SYM const std::vector<double> matlabToStdVector<double>(
    const mxArray* in) {
  // Works for both row vectors and column vectors.
  if (mxGetM(in) != 1 && mxGetN(in) != 1) {
    throw std::runtime_error("Not a vector.");
  }

  double* data = mxGetPrSafe(in);
  return std::vector<double>(data, data + mxGetNumberOfElements(in));
}

mwSize sub2ind(mwSize ndims, const mwSize* dims, const mwSize* sub) {
  mwSize stride = 1;
  mwSize ret = 0;

  for (mwSize i = 0; i < ndims; i++) {
    ret += sub[i] * stride;
    stride *= dims[i];
  }

  return ret;
}

void sizecheck(const mxArray* mat, mwSize M, mwSize N) {
  if (mxGetM(mat) != M) {
    mexErrMsgIdAndTxt("Drake:WrongSize",
                      "wrong number of rows. Expected: %d but got: %d.", M,
                      mxGetM(mat));
  }

  if (mxGetN(mat) != N) {
    mexErrMsgIdAndTxt("Drake:WrongSize",
                      "wrong number of columns. Expected: %d but got: %d.", N,
                      mxGetN(mat));
  }
}

// Builds a MATLAB sparse matrix in MEX from a given Eigen matrix. The caller
// is responsible for destroying the resulting array.
template <typename Derived>
mxArray* eigenToMatlabSparse(Eigen::MatrixBase<Derived> const& M,
                             int* num_non_zero) {
  const mwSize rows = M.rows();
  const mwSize cols = M.cols();

  std::vector<mwIndex> ir;
  std::vector<mwIndex> jc;
  std::vector<double> pr;

  int cumulative_nonzero = 0;
  jc.push_back(cumulative_nonzero);
  const double eps = std::numeric_limits<double>::epsilon();

  for (mwIndex j = 0; j < cols; j++) {
    for (mwIndex i = 0; i < rows; i++) {
      const double value = M(i, j);

      if (value > eps || value < -eps) {
        ir.push_back(i);
        pr.push_back(value);
        cumulative_nonzero++;
      }
    }

    jc.push_back(cumulative_nonzero);
  }

  mxArray* sparse_mex = mxCreateSparse(rows, cols, cumulative_nonzero, mxREAL);

  std::memcpy(mxGetPr(sparse_mex), pr.data(),
              cumulative_nonzero * sizeof(double));
  std::memcpy(reinterpret_cast<int*>(mxGetIr(sparse_mex)), ir.data(),
              cumulative_nonzero * sizeof(mwIndex));
  std::memcpy(reinterpret_cast<int*>(mxGetJc(sparse_mex)), jc.data(),
              (cols + 1) * sizeof(mwIndex));

  *num_non_zero = cumulative_nonzero;
  return sparse_mex;
}

template DLL_EXPORT_SYM mxArray* eigenToMatlabSparse(
    Eigen::MatrixBase<Eigen::MatrixXd> const&, int*);

template DLL_EXPORT_SYM mxArray* eigenToMatlabSparse(
    Eigen::MatrixBase<Eigen::Map<Eigen::MatrixXd>> const&, int*);

// template DLL_EXPORT_SYM const std::vector<double> matlabToStdVector<double>(
//     const mxArray* in); already explicitly specialized

template DLL_EXPORT_SYM const std::vector<int> matlabToStdVector<int>(
    const mxArray* in);

template DLL_EXPORT_SYM const std::vector<Eigen::Index>
matlabToStdVector<Eigen::Index>(const mxArray* in);

template DLL_EXPORT_SYM const std::vector<bool> matlabToStdVector<bool>(
    const mxArray* in);
