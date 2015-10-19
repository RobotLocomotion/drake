#include "drakeMexUtil.h"
#include <stdexcept>

using namespace std;
using namespace Eigen;

bool isa(const mxArray* mxa, const char* class_str)
// mxIsClass seems to not be able to handle derived classes. so i'll implement what I need by calling back to matlab
{
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

bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* filename)
{
  int i;
  mxArray* ex = mexCallMATLABWithTrap(nlhs, plhs, nrhs, prhs, filename);
  if (ex) {
    mexPrintf("DrakeSystem S-Function: error when calling ''%s'' with the following arguments:\n", filename);
    for (i = 0; i < nrhs; i++)
      mexCallMATLAB(0, NULL, 1, &prhs[i], "disp");
    mxArray *report;
    mexCallMATLAB(1, &report, 1, &ex, "getReport");
    char *errmsg = mxArrayToString(report);
    mexPrintf(errmsg);
    mxFree(errmsg);
    mxDestroyArray(report);
    mexErrMsgIdAndTxt("Drake:mexCallMATLABsafe:CallbackError", "Error in MATLAB callback.\nSee additional debugging information above");
    mxDestroyArray(ex);
    return true;
  }
  for (i = 0; i < nlhs; i++)
    if (!plhs[i]) {
      mexPrintf("Drake mexCallMATLABsafe: error when calling ''%s'' with the following arguments:\n", filename);
      for (i = 0; i < nrhs; i++)
        mexCallMATLAB(0, NULL, 1, &prhs[i], "disp");
      mexErrMsgIdAndTxt("Drake:mexCallMATLABsafe:NotEnoughOutputs", "Asked for %d outputs, but function only returned %d\n", nrhs, i);
      return true;
    }
  return false;
}

/*
 * @param subclass_name (optional) if you want to call a class that derives from
 * DrakeMexPointer (e.g. so that you can refer to it as something more specific in
 * your matlab code), then you can pass in the alternative name here.  The constructor
 * for this class must take the same inputs as the DrakeMexPointer constructor.
 */
mxArray* createDrakeMexPointer(void* ptr, const std::string&  name, int num_additional_inputs, mxArray* delete_fcn_additional_inputs[], const std::string&  subclass_name, const std::string& mex_function_name_prefix)
{
  mxClassID cid;
  if (sizeof(ptr) == 4) cid = mxUINT32_CLASS;
  else if (sizeof(ptr) == 8) cid = mxUINT64_CLASS;
  else mexErrMsgIdAndTxt("Drake:constructDrakeMexPointer:PointerSize", "Are you on a 32-bit machine or 64-bit machine??");

  int nrhs = 3 + num_additional_inputs;
  mxArray *plhs[1];
  mxArray **prhs;  
  prhs = new mxArray*[nrhs];

  prhs[0] = mxCreateNumericMatrix(1, 1, cid, mxREAL);
  memcpy(mxGetData(prhs[0]), &ptr, sizeof(ptr));

  prhs[1] = mxCreateString((mex_function_name_prefix+mexFunctionName()).c_str());

  prhs[2] = mxCreateString(name.c_str());

  for (int i = 0; i < num_additional_inputs; i++)
    prhs[3+i] = delete_fcn_additional_inputs[i];

//  mexPrintf("deleteMethod = %s\n name =%s\n", deleteMethod,name);

  // call matlab to construct mex pointer object
  if (!subclass_name.empty()) {
    mexCallMATLABsafe(1, plhs, nrhs, prhs, subclass_name.c_str());
    if (!isa(plhs[0], "DrakeMexPointer")) {
      mxDestroyArray(plhs[0]);
      mexErrMsgIdAndTxt("Drake:createDrakeMexPointer:InvalidSubclass", "subclass_name is not a valid subclass of DrakeMexPointer");
    }
  }
  else
    mexCallMATLABsafe(1, plhs, nrhs, prhs, "DrakeMexPointer");

  mexLock();

  delete[] prhs;
  return plhs[0];
}


void* getDrakeMexPointer(const mxArray* mx)
{
  if (!mx)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs", "null mxArray");

  void* ptr = NULL;

  // todo: optimize this by caching the pointer values, as described in
  // http://groups.csail.mit.edu/locomotion/bugs/show_bug.cgi?id=1590
  mxArray* ptrArray = mxGetProperty(mx, 0, "ptr");

  if (!ptrArray)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs", "cannot retrieve 'ptr' field from this mxArray.  are you sure it's a valid DrakeMexPointer object?");

  switch (sizeof(void*)) { 
    case 4:
      if (!mxIsUint32(ptrArray))
        mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadPointerSize", "DrakeMexPointer expected a 32-bit ptr field but got something else");        
      break;
    case 8:
      if (!mxIsUint64(ptrArray))
        mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadPointerSize", "DrakeMexPointer expected a 64-bit ptr field but got something else");        
      break;
    default:
      mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadPointerSize", "DrakeMexPointer got a pointer that was neither 32-bit nor 64-bit.");
  }

  if (!mxIsNumeric(ptrArray) || mxGetNumberOfElements(ptrArray) != 1)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs","the ptr property of this DrakeMexPointer does not appear to contain a valid pointer");
  memcpy(&ptr, mxGetData(ptrArray), sizeof(ptr));     // note: could use a reinterpret_cast here instead

  return ptr;
}


std::string mxGetStdString(const mxArray* array) {
  mwSize buffer_length = mxGetNumberOfElements(array) + 1;
  char* buffer = new char[buffer_length];
  int status = mxGetString(array, buffer, buffer_length);

  if (status != 0) {
    delete[] buffer;
    throw runtime_error("mxGetStdString failed. Possible cause: mxArray is not a string array.");
  }
  else {
    string ret(buffer);
    delete[] buffer;
    return ret;
  }
}

std::vector<std::string> mxGetVectorOfStdStrings(const mxArray* array) {
  if (!mxIsCell(array))
    throw runtime_error("the input is not a cell array");

  std::vector<std::string> strings;
  size_t numel = mxGetNumberOfElements(array);
  for (size_t i=0; i<numel; i++) {
    strings.push_back(mxGetStdString(mxGetCell(array,i)));
  }
  return strings;
}

mxArray* stdStringToMatlab(const std::string& str)
{
  return mxCreateString(str.c_str());
}

mxArray* vectorOfStdStringsToMatlab(const std::vector<std::string>& strs) {
  mxArray* cell = mxCreateCellMatrix(strs.size(),1);
  for (int i=0; i<strs.size(); i++) {
    mxSetCell(cell,i,mxCreateString(strs[i].c_str()));
  }
  return cell;
}




double * mxGetPrSafe(const mxArray *pobj) {
  if (!mxIsDouble(pobj)) mexErrMsgIdAndTxt("Drake:mxGetPrSafe:BadInputs", "mxGetPr can only be called on arguments which correspond to Matlab doubles");
  return mxGetPr(pobj);
}

mxArray* mxGetPropertySafe(const mxArray* array, std::string const& field_name) {
  return mxGetPropertySafe(array, 0, field_name);
}

mxArray* mxGetPropertySafe(const mxArray* array, size_t index, std::string const& field_name)
{
  mxArray* ret = mxGetProperty(array, index, field_name.c_str());
  if (!ret)
  {
    mexErrMsgIdAndTxt("Drake:mxGetPropertySafe", ("Property not found: " + field_name).c_str());
  }
  return ret;
}

mxArray* mxGetFieldSafe(const mxArray* array, std::string const& field_name) {
  return mxGetFieldSafe(array, 0, field_name);
}

mxArray* mxGetFieldSafe(const mxArray* array, size_t index, std::string const& field_name)
{
  mxArray* ret = mxGetField(array, index, field_name.c_str());
  if (!ret)
  {
    mexErrMsgIdAndTxt("Drake:mxGetFieldSafe", ("Field not found: " + field_name).c_str());
  }
  return ret;
}

void mxSetFieldSafe(mxArray* array, size_t index, std::string const & fieldname, mxArray* data)
{
  int fieldnum;
  fieldnum = mxGetFieldNumber(array, fieldname.c_str());
  if (fieldnum < 0) {
    fieldnum = mxAddField(array, fieldname.c_str());
  }
  mxSetFieldByNumber(array, index, fieldnum, data);
}

mxArray* mxGetFieldOrPropertySafe(const mxArray* array, std::string const& field_name) {
  return mxGetFieldOrPropertySafe(array, 0, field_name);
}

mxArray* mxGetFieldOrPropertySafe(const mxArray* array, size_t index, std::string const& field_name) {
  mxArray* field_or_property = mxGetField(array, index, field_name.c_str());
  if (field_or_property == nullptr) {
    field_or_property = mxGetPropertySafe(array, index, field_name);
  }
  return field_or_property;
}

template <typename T>
const std::vector<T> matlabToStdVector(const mxArray* in) {
  // works for both row vectors and column vectors

  if (mxGetNumberOfElements(in) == 0) // if input is empty, output is an empty vector
    return std::vector<T>();

  if (mxGetM(in) != 1 && mxGetN(in) != 1)
    throw std::runtime_error("Not a vector");
  std::vector<T> ret;
  if (mxIsLogical(in)) {
    mxLogical* data = mxGetLogicals(in);
    for (int i = 0; i < mxGetNumberOfElements(in); i++) {
      ret.push_back(static_cast<T>(data[i]));
    }
  }
  else {
    double* data = mxGetPrSafe(in);
    for (int i = 0; i < mxGetNumberOfElements(in); i++) {
      ret.push_back(static_cast<T>(data[i]));
    }
  }
  return ret;
}

template <>
DLLEXPORT const std::vector<double> matlabToStdVector<double>(const mxArray* in) {
  // works for both row vectors and column vectors
  if (mxGetM(in) != 1 && mxGetN(in) != 1)
    throw std::runtime_error("Not a vector");
  double* data = mxGetPrSafe(in);
  return std::vector<double>(data, data + mxGetNumberOfElements(in));
}

DLLEXPORT Matrix<Polynomiald, Dynamic, Dynamic> msspolyToEigen(const mxArray* msspoly)
{
  auto dim = matlabToEigenMap<1,2>(mxGetPropertySafe(msspoly,0,"dim"));
  auto sub = matlabToEigenMap<Dynamic,2>(mxGetPropertySafe(msspoly,0,"sub"));
  auto var = matlabToEigenMap<Dynamic,Dynamic>(mxGetPropertySafe(msspoly,0,"var"));
  auto pow = matlabToEigenMap<Dynamic,Dynamic>(mxGetPropertySafe(msspoly,0,"pow"));
  auto coeff = matlabToEigenMap<Dynamic,1>(mxGetPropertySafe(msspoly,0,"coeff"));

  assert(sub.rows()==var.rows());
  assert(sub.rows()==pow.rows());
  assert(sub.rows()==coeff.rows());
  assert(var.cols()==pow.cols());

  Matrix<Polynomiald, Dynamic, Dynamic> poly((int)dim(0),(int)dim(1));
  for (int i=0; i<sub.rows(); i++) {
    vector<Polynomiald::Term> terms;
    int j=0;
    while (j<var.cols() && var(i,j)>0) {
      Polynomiald::Term t;
      t.var = (Polynomiald::VarType) var(i,j);
      t.power = (Polynomiald::PowerType) pow(i,j);
      terms.push_back(t);
      j++;
    }
    Polynomiald p(coeff(i),terms);
    poly((DenseIndex) sub(i,0)-1,(DenseIndex) sub(i,1)-1) += p;
  }

//  cout << poly << endl;

  return poly;
}

DLLEXPORT Eigen::Matrix<TrigPolyd, Eigen::Dynamic, Eigen::Dynamic> trigPolyToEigen(const mxArray* trigpoly)
{
  auto q = msspolyToEigen(mxGetPropertySafe(trigpoly,0,"q"));
  auto s = msspolyToEigen(mxGetPropertySafe(trigpoly,0,"s"));
  auto c = msspolyToEigen(mxGetPropertySafe(trigpoly,0,"c"));
  auto p = msspolyToEigen(mxGetPropertySafe(trigpoly,0,"p"));

  TrigPolyd::SinCosMap m;
  for (int i=0; i<q.size(); i++) {
    TrigPolyd::SinCosVars sc;
    sc.s = s(i).getSimpleVariable();
    sc.c = c(i).getSimpleVariable();
    m[q(i).getSimpleVariable()] = sc;
  }

  // todo: feels very inefficient (one copy of the sincosmap for every element of the matrix).
  // consider using shared_ptrs for the sincosmap instead.
  Matrix<TrigPolyd, Dynamic, Dynamic> tp(p.rows(),p.cols());
  for (int i=0; i<p.size(); i++) {
    tp(i) = TrigPolyd(p(i),m);
  }

  return tp;
}


mwSize sub2ind(mwSize ndims, const mwSize* dims, const mwSize* sub) {
  mwSize stride = 1;
  mwSize ret = 0;
  for (int i = 0; i < ndims; i++) {
    ret += sub[i] * stride;
    stride *= dims[i];
  }
  return ret;
}

void sizecheck(const mxArray* mat, mwSize M, mwSize N) {
  if (mxGetM(mat) != M) {
    mexErrMsgIdAndTxt("Drake:WrongSize", "wrong number of rows. Expected: %d but got: %d", M, mxGetM(mat));
  }
  if (mxGetN(mat) != N) {
    mexErrMsgIdAndTxt("Drake:WrongSize", "wrong number of columns. Expected: %d but got: %d", N, mxGetN(mat));
  }
  return;
}

//builds a matlab sparse matrix in mex from a given eigen matrix
//the caller is responsible for destroying the resulting array
template <typename Derived>
mxArray* eigenToMatlabSparse(MatrixBase<Derived> const & M, int & num_non_zero) 
{
  const mwSize rows = M.rows();
  const mwSize cols = M.cols();

  vector<mwIndex> ir;
  vector<mwIndex> jc;
  vector<double> pr;

  int cumulative_nonzero = 0;
  jc.push_back(cumulative_nonzero);
  double eps = std::numeric_limits<double>::epsilon();
  for (mwIndex j = 0; j < cols; j++) {
    for (mwIndex i = 0; i < rows; i++)  {
      double value = M(i, j);
      
      if (value > eps || value < -eps) {
        ir.push_back(i);
        pr.push_back(value);
        cumulative_nonzero++;
      }
    }
    jc.push_back(cumulative_nonzero);
  }

  mxArray* sparse_mex = mxCreateSparse(rows, cols, cumulative_nonzero, mxREAL);
  
  memcpy((double*)mxGetPr(sparse_mex), pr.data(), cumulative_nonzero * sizeof(double));
  memcpy((int*)mxGetIr(sparse_mex), ir.data(), cumulative_nonzero * sizeof(mwIndex));
  memcpy((int *)mxGetJc(sparse_mex), jc.data(), (cols + 1) * sizeof(mwIndex));

  num_non_zero = cumulative_nonzero;
  return sparse_mex;
}

template DLLEXPORT mxArray* eigenToMatlabSparse(MatrixBase< MatrixXd > const &, int &) ;
template DLLEXPORT mxArray* eigenToMatlabSparse(MatrixBase< Map< MatrixXd> > const &, int &) ;
// template DLLEXPORT const std::vector<double> matlabToStdVector<double>(const mxArray* in); already explicitly specialized
template DLLEXPORT const std::vector<int> matlabToStdVector<int>(const mxArray* in);
template DLLEXPORT const std::vector<Eigen::DenseIndex> matlabToStdVector<Eigen::DenseIndex>(const mxArray* in);
template DLLEXPORT const std::vector<bool> matlabToStdVector<bool>(const mxArray* in);
//template DLLEXPORT mxArray* eigenToMSSPoly(const Matrix<Polynomiald,Dynamic,Dynamic> & poly);
