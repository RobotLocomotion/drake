#include <mex.h>
#include <cstring>
#include <map>
#include <vector>

#include "lcm/lcm-cpp.hpp"

#include "drake/lcmt_call_matlab.hpp"
#include "drake/common/drake_assert.h"
#include "drake/matlab/util/drakeMexUtil.h"

// Mex client for matlab feval

void flushMatlabEventBuffer(void) {
  // an ugly hack.  drawnow used to work, but the internet confirms that it
  // stopped working around R2015.
  static mxArray* time = mxCreateDoubleScalar(0.001);
  mexCallMATLAB(0, nullptr, 1, &time, "pause");
}

class Handler {
  // TODO(russt): Implement a destructor message (or string) for the local vars
  // which calls mxDestroyArray

 public:
  ~Handler() {}  // Note: could mxDestroyArray the client_vars_, but this method
                 // will never actually be called.

  void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                     const drake::lcmt_call_matlab* msg) {
    int i;
    std::vector<mxArray *> lhs(msg->nlhs), rhs(msg->nrhs);

    // Create the input arguments
    for (i = 0; i < msg->nrhs; i++) {
      if (msg->rhs[i].num_bytes == 0) {
        mexPrintf("rhs %d seems to have zero bytes.  dropping message %s.\n", i, msg->function_name.c_str());
        mexPrintf("type: %d\n", msg->rhs[i].type);
        mexPrintf("rows: %d\n", msg->rhs[i].rows);
        mexPrintf("cols: %d\n", msg->rhs[i].cols);
        mexPrintf("num_bytes: %d\n", msg->rhs[i].num_bytes);
        for (int j=0; j<i; j++) {
          mxDestroyArray(rhs[j]);
        }
        return;
      }

      switch (msg->rhs[i].type) {
        case drake::lcmt_matlab_array::REMOTE_VARIABLE_REFERENCE: {
          int64_t id;
          DRAKE_DEMAND(sizeof(int64_t) == msg->rhs[i].num_bytes);
          memcpy(&id, msg->rhs[i].data.data(), msg->rhs[i].num_bytes);
          if (client_vars_.find(id) == client_vars_.end()) {
            mexWarnMsgTxt(
                "rhs referenced unknown local variable.  dropping message.");
            for (int j=0; j<i; j++) {
              mxDestroyArray(rhs[j]);
            }
            return;
          }
          rhs[i] = client_vars_.at(id);
          break;
        }
        case drake::lcmt_matlab_array::DOUBLE: {
          rhs[i] =
              mxCreateDoubleMatrix(msg->rhs[i].rows, msg->rhs[i].cols, mxREAL);
          DRAKE_DEMAND(sizeof(double)*msg->rhs[i].rows*msg->rhs[i].cols == msg->rhs[i].num_bytes);
          memcpy(mxGetPr(rhs[i]), msg->rhs[i].data.data(),
                 msg->rhs[i].num_bytes);
          break;
        }
        case drake::lcmt_matlab_array::CHAR: {
          mwSize dims[2];
          dims[0] = msg->rhs[i].rows;
          dims[1] = msg->rhs[i].cols;
          rhs[i] = mxCreateCharArray(2, dims);
          mxChar* char_data = static_cast<mxChar*>(mxGetData(rhs[i]));
          DRAKE_DEMAND(msg->rhs[i].rows*msg->rhs[i].cols == msg->rhs[i].data.size());
          for (int j = 0; j < dims[0] * dims[1]; j++) { // Note: sizeof(mxChar) == 2.
            char_data[j] = static_cast<mxChar>(msg->rhs[i].data[j]);
          }
          break;
        }
        case drake::lcmt_matlab_array::LOGICAL: {
          rhs[i] = mxCreateLogicalMatrix(msg->rhs[i].rows, msg->rhs[i].cols);
          DRAKE_DEMAND(msg->rhs[i].rows*msg->rhs[i].cols == msg->rhs[i].data.size());
          mxLogical* logical_data = static_cast<mxLogical*>(mxGetData(rhs[i]));
          for (int j = 0; j < msg->rhs[i].data.size(); j++) {
            logical_data[j] = static_cast<mxLogical>(msg->rhs[i].data[j]);
          }
          break;
        }
        default:
          mexWarnMsgTxt("Unknown rhs variable type.");
          for (int j=0; j<i; j++) {
            mxDestroyArray(rhs[j]);
          }
          return;
      }
    }

    // Make the actual call to MATLAB.
    mexCallMATLABsafe(static_cast<int>(lhs.size()), lhs.data(),
                  static_cast<int>(rhs.size()), rhs.data(),
                  msg->function_name.c_str());

    // Assign any local variables that were returned by this call.
    for (i = 0; i < msg->nlhs; i++) {
      // Zap any old variables that will be overwritten by this call
      if (client_vars_.find(msg->lhs[i]) != client_vars_.end())
        mxDestroyArray(client_vars_[msg->lhs[i]]);
      client_vars_[msg->lhs[i]] = lhs[i];
    }

    // Clean up the input argument data.
    for (i = 0; i < rhs.size(); i++) {
      mxDestroyArray(rhs[i]);
    }
  }

 private:
  std::map<int64_t, mxArray*> client_vars_;
};

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  lcm::LCM lcm;
  if (!lcm.good()) mexErrMsgTxt("Bad LCM reference.");

  // TODO(russt): take channel name as an optional input
  Handler handler_object;
  std::string channel = "LCM_CALL_MATLAB";
  lcm.subscribe(channel, &Handler::handleMessage, &handler_object);

  mexPrintf("Listening for messages on LCM_CALL_MATLAB...\n");
  flushMatlabEventBuffer();

  while (1) {
    int n = lcm.handleTimeout(10);
    if (n < 0) {
      mexErrMsgTxt("Something went wrong in lcm.handle.");
    } else if (n == 0) {  // no messages this time, let matlab refresh
      flushMatlabEventBuffer();
    }  // else I got a message, so do nothing
  }

  mexPrintf("Cleaning up.\n");
}
