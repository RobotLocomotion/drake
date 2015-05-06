#include "mex.h"
#include "QPCommon.h"

DLLEXPORT std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> parseSupportData(const mxArray* supp_data);

std::shared_ptr<drake::lcmt_qp_controller_input> encodeQPInputLCM(const mxArray *qp_input);

PiecewisePolynomial<double> matlabToPiecewisePolynomial(const mxArray* pobj, int index); 


void parseRobotPropertyCache(const mxArray *rpc_obj, RobotPropertyCache *rpc);
