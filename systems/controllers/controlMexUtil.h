#include "mex.h"
#include "controlUtil.h"

DLLEXPORT std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> parseSupportData(const mxArray* supp_data);
