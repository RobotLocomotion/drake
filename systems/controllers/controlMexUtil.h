#include "mex.h"
#include "QPCommon.h"

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeControlMexUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif

DLLEXPORT std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> parseSupportData(const mxArray* supp_data);

DLLEXPORT std::shared_ptr<drake::lcmt_qp_controller_input> encodeQPInputLCM(const mxArray *qp_input);

DLLEXPORT PiecewisePolynomial<double> matlabToPiecewisePolynomial(const mxArray* pobj, int index); 

DLLEXPORT void parseRobotPropertyCache(const mxArray *rpc_obj, RobotPropertyCache *rpc);

DLLEXPORT mxArray* myGetProperty(const mxArray* pobj, const char* propname);
DLLEXPORT mxArray* myGetField(const mxArray* pobj, const char* propname);
DLLEXPORT mxArray* myGetField(const mxArray* pobj, const int idx, const char* propname);
// convert Matlab cell array of strings into a C++ vector of strings
DLLEXPORT std::vector<std::string> get_strings(const mxArray *rhs);
