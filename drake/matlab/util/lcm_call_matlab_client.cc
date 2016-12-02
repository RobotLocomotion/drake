#include <mex.h>
#include <cstring>
#include <map>
#include <vector>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/lcmt_call_matlab.hpp"
#include "drake/matlab/util/drakeMexUtil.h"

// Mex client for matlab feval.

// TODO(russt): Implement some form of security, like only accepting messages
// from localhost by default.  Perhaps writing/sending a secret password file to
// disk which is only accessible to localhost (e.g. .Xauthority) would work.

// Known issues: Some functions in matlab do not like to get called from mex.
// For instance, calling "histogram" fails with "Error using inputname...
// Argument number is not valid." because it is trying to extract a variable
// name from the matlab calling stack (and there are none).  Bad matlab.

namespace {

void FlushMatlabEventBuffer(void) {
  // An ugly hack.  drawnow used to work, but the internet confirms that it
  // stopped working around R2015.
  static mxArray* time = mxCreateDoubleScalar(0.001);
  mexCallMATLAB(0, nullptr, 1, &time, "pause");
}

// Per discussion at https://github.com/RobotLocomotion/drake/pull/4256 ,
// guard against incompatibility between lcm and matlab.
// Note: Candidate for moving to a shared location for reuse.
void CheckIfLcmWillExplode() {
  mxArray* plhs[1];
  mxArray* prhs[2];

  // Only known to fail on unix.
  mexCallMATLAB(1, plhs, 0, nullptr, "isunix");
  bool is_unix = mxIsLogicalScalarTrue(plhs[0]);
  mxDestroyArray(plhs[0]);
  if (!is_unix) return;

  // Fails on MATLAB > 2014a
  prhs[0] = mxCreateString("matlab");
  prhs[1] = mxCreateString("2014b");
  mexCallMATLAB(1, plhs, 2, prhs, "verLessThan");
  bool is_safe_matlab_version = mxIsLogicalScalarTrue(plhs[0]);
  mxDestroyArray(plhs[0]);
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  if (is_safe_matlab_version) return;

  mexErrMsgTxt(
      "LCM mex is incompatible with MATLAB >R2014a on unix.  See "
      "https://github.com/lcm-proj/lcm/issues/147 .");
}

class Handler {
  // TODO(russt): Implement an interface that allows the remote publisher to
  // manually delete a client variable.

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
        mexPrintf("rhs %d seems to have zero bytes.  dropping message %s.\n", i,
                  msg->function_name.c_str());
        mexPrintf("type: %d\n", msg->rhs[i].type);
        mexPrintf("rows: %d\n", msg->rhs[i].rows);
        mexPrintf("cols: %d\n", msg->rhs[i].cols);
        mexPrintf("num_bytes: %d\n", msg->rhs[i].num_bytes);
        for (int j = 0; j < i; j++) {
          mxDestroyArray(rhs[j]);
        }
        return;
      }

      switch (msg->rhs[i].type) {
        case drake::lcmt_matlab_array::REMOTE_VARIABLE_REFERENCE: {
          int64_t id;
          DRAKE_DEMAND(static_cast<int>(sizeof(int64_t)) ==
                       msg->rhs[i].num_bytes);
          memcpy(&id, msg->rhs[i].data.data(), msg->rhs[i].num_bytes);
          if (client_vars_.find(id) == client_vars_.end()) {
            mexWarnMsgTxt(
                "rhs referenced unknown local variable.  dropping message.");
            for (int j = 0; j < i; j++) {
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
          DRAKE_DEMAND(static_cast<int>(sizeof(double)) * msg->rhs[i].rows *
                           msg->rhs[i].cols ==
                       msg->rhs[i].num_bytes);
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
          DRAKE_DEMAND(msg->rhs[i].rows * msg->rhs[i].cols ==
                       static_cast<int>(msg->rhs[i].data.size()));
          for (int j = 0; j < static_cast<int>(dims[0] * dims[1]);
               j++) {  // Note: sizeof(mxChar) == 2.
            char_data[j] = static_cast<mxChar>(msg->rhs[i].data[j]);
          }
          break;
        }
        case drake::lcmt_matlab_array::LOGICAL: {
          rhs[i] = mxCreateLogicalMatrix(msg->rhs[i].rows, msg->rhs[i].cols);
          DRAKE_DEMAND(msg->rhs[i].rows * msg->rhs[i].cols ==
                       static_cast<int>(msg->rhs[i].data.size()));
          mxLogical* logical_data = static_cast<mxLogical*>(mxGetData(rhs[i]));
          for (int j = 0; j < static_cast<int>(msg->rhs[i].data.size()); j++) {
            logical_data[j] = static_cast<mxLogical>(msg->rhs[i].data[j]);
          }
          break;
        }
        default:
          mexWarnMsgTxt("Unknown rhs variable type.");
          for (int j = 0; j < i; j++) {
            mxDestroyArray(rhs[j]);
          }
          return;
      }
    }

    // Make the actual call to MATLAB.
    bool trapped_error = mexCallMATLABsafe(
        static_cast<int>(lhs.size()), lhs.data(), static_cast<int>(rhs.size()),
        rhs.data(), msg->function_name.c_str());

    if (!trapped_error) {
      // Assign any local variables that were returned by this call.
      for (i = 0; i < msg->nlhs; i++) {
        // Zap any old variables that will be overwritten by this call.
        if (client_vars_.find(msg->lhs[i]) != client_vars_.end())
          mxDestroyArray(client_vars_[msg->lhs[i]]);
        client_vars_[msg->lhs[i]] = lhs[i];
      }
    }

    // Clean up the input argument data.
    for (i = 0; i < static_cast<int>(rhs.size()); i++) {
      // Make sure it's not a client var.
      if (msg->rhs[i].type !=
          drake::lcmt_matlab_array::REMOTE_VARIABLE_REFERENCE)
        mxDestroyArray(rhs[i]);
    }
  }

 private:
  std::map<int64_t, mxArray*> client_vars_;
};

}  // end namespace

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  CheckIfLcmWillExplode();  // Run-time compatibility check.

  lcm::LCM lcm;
  if (!lcm.good()) mexErrMsgTxt("Bad LCM reference.");

  // TODO(russt): Take channel name as an optional input.
  Handler handler_object;
  std::string channel = "LCM_CALL_MATLAB";
  lcm.subscribe(channel, &Handler::handleMessage, &handler_object);

  mexPrintf("Listening for messages on LCM_CALL_MATLAB...\n");
  FlushMatlabEventBuffer();

  while (1) {
    int n = lcm.handleTimeout(10);
    if (n < 0) {
      mexErrMsgTxt("Something went wrong in lcm.handle.");
    } else if (n == 0) {  // No messages this time, let matlab refresh.
      FlushMatlabEventBuffer();
    }  // Else I got a message, so do nothing.
  }

  mexPrintf("Cleaning up.\n");  // Note: Expect to never get here.
}
