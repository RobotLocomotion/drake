#include <cstring>
#include <map>
#include <mex.h>
#include <vector>

#include "lcm/lcm-cpp.hpp"

#include "drake/lcmt_matlab_feval.hpp"


/// Mex client for matlab feval

class Handler {
 public:
  ~Handler() {}

  void handleMessage(const lcm::ReceiveBuffer* rbuf,
                   const std::string& chan,
                   const drake::lcmt_matlab_feval* msg) {
    int i;
    std::vector<mxArray*> lhs(msg->nlhs), rhs(msg->nrhs);

    // Create the input arguments
    for (i=0; i<msg->nrhs; i++) {
      switch (msg->rhs[i].type) {
        case drake::lcmt_matlab_array::REMOTE_VARIABLE_REFERENCE:
        {
          int64_t id;
          memcpy(&id,msg->rhs[i].data.data(),sizeof(int64_t));
          if (client_vars.find(id) == client_vars.end()) {
            mexWarnMsgTxt("rhs referenced unknown local variable.  dropping message.");
            return;
          }
          rhs[i] = client_vars[id];
          break;
        }
        case drake::lcmt_matlab_array::DOUBLE:
        {
          rhs[i] = mxCreateDoubleMatrix(msg->rhs[i].rows, msg->rhs[i].cols, mxREAL);
          memcpy(mxGetPr(rhs[i]),msg->rhs[i].data.data(),sizeof(double) * msg->rhs[i].rows * msg->rhs[i].cols);
          break;
        }
        case drake::lcmt_matlab_array::CHAR:
        {
          mwSize dims[2];
          dims[0] = msg->rhs[i].rows;
          dims[1] = msg->rhs[i].cols;
          rhs[i] = mxCreateCharArray(2,dims);
          memcpy(mxGetData(rhs[i]),msg->rhs[i].data.data(),sizeof(char) * dims[0] * dims[1]);
          break;
        }
        case drake::lcmt_matlab_array::LOGICAL:
        {
          rhs[i] = mxCreateLogicalMatrix(msg->rhs[i].rows, msg->rhs[i].cols);
          memcpy(mxGetData(rhs[i]),msg->rhs[i].data.data(), msg->rhs[i].num_bytes);
          break;
        }
        default:
          mexWarnMsgTxt("Unknown rhs variable type.");
          return;
      }
    }

    // Find (or create) any local variables that will be returned by this call.
    for (i=0; i<msg->nlhs; i++) {
      if (client_vars.find(msg->lhs[i]) == client_vars.end()) { // then construct a new local variable
        client_vars[msg->lhs[i]] = nullptr;
      }
      lhs[i] = client_vars[msg->lhs[i]];
    }

    // TODO(russt): Implement a destructor for the local vars which calls mxDestroyArray

    // Make the actual call to MATLAB.
    mexCallMATLAB(static_cast<int>(lhs.size()), lhs.data(),
                  static_cast<int>(rhs.size()), rhs.data(),
                  msg->function_name.c_str());

    // Clean up the input argument data.
    for (i=0; i<rhs.size(); i++) { mxDestroyArray(rhs[i]); }
  }

 private:
  std::map<int64_t,mxArray*> client_vars;
};


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  lcm::LCM lcm;
  if (!lcm.good()) mexErrMsgTxt("Bad LCM reference.");

  // TODO(russt): take channel name as an optional input
  Handler handler_object;
  lcm.subscribe("LCM_MATLAB_FEVAL", &Handler::handleMessage, &handler_object);

  while (1) {
    if (lcm.handle() != 0) {
      mexErrMsgTxt("Something went wrong in lcm.handle.");
    }
  }
}