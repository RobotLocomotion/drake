#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <mex.h>
#include <signal.h>
#include <sys/select.h>

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

// TODO(russt): Implement an interface that allows the remote publisher to
// manually delete a client variable.

namespace {

void FlushMatlabEventBuffer(void) {
  // An ugly hack.  drawnow used to work, but the internet confirms that it
  // stopped working around R2015.
  mxArray* time = mxCreateDoubleScalar(0.001);
  mexCallMATLABsafe(0, nullptr, 1, &time, "pause");
  mxDestroyArray(time);
}

// Per discussion at https://github.com/RobotLocomotion/drake/pull/4256 ,
// guard against incompatibility between lcm and matlab.
// Note: Candidate for moving to a shared location for reuse.
void CheckIfLcmWillExplode() {
  mxArray* plhs[1];
  mxArray* prhs[2];

  // Only known to fail on unix.
  mexCallMATLABsafe(1, plhs, 0, nullptr, "isunix");
  bool is_unix = mxIsLogicalScalarTrue(plhs[0]);
  mxDestroyArray(plhs[0]);
  if (!is_unix) return;

  // Fails on MATLAB > 2014a
  prhs[0] = mxCreateString("matlab");
  prhs[1] = mxCreateString("2014b");
  mexCallMATLABsafe(1, plhs, 2, prhs, "verLessThan");
  bool is_safe_matlab_version = mxIsLogicalScalarTrue(plhs[0]);
  mxDestroyArray(plhs[0]);
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  if (is_safe_matlab_version) return;

  mexErrMsgTxt(
      "LCM mex is incompatible with MATLAB >R2014a on unix.  See "
      "https://github.com/lcm-proj/lcm/issues/147 .");
}

class LCMLoop {
 public:
  explicit LCMLoop(lcm::LCM* lcm) : lcm_(lcm), stop_(false) {}

  void Stop() { stop_ = true; }

  void LoopWithSelect() {
    const double timeoutInSeconds = 0.3;

    int lcm_file_descriptor = lcm_->getFileno();

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeoutInSeconds * 1e6;

    fd_set fds;

    while (!stop_) {
      FD_ZERO(&fds);
      FD_SET(lcm_file_descriptor, &fds);
      int status = select(lcm_file_descriptor + 1, &fds, 0, 0, &tv);
      int error_code = errno;
      if (status == -1 && error_code != EINTR) {
        std::cout << "select() returned error: " << error_code << std::endl;
        break;
      } else if (status == -1 && error_code == EINTR) {
        std::cout << "select() interrupted" << std::endl;
        break;
      }
      if (status > 0 && FD_ISSET(lcm_file_descriptor, &fds)) {
        if (lcm_->handle() != 0) {
          std::cout << "lcm->handle() returned non-zero" << std::endl;
          break;
        }
      }
    }
  }

 private:
  lcm::LCM* lcm_;
  bool stop_{false};
};

class Handler {
 public:
  Handler(std::queue<std::unique_ptr<drake::lcmt_call_matlab>>* message_queue,
          std::mutex* message_mutex)
      : message_queue_(message_queue), message_mutex_(message_mutex) {}

  ~Handler() {}

  void HandleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan) {
    std::lock_guard<std::mutex> lock(*message_mutex_);
    message_queue_->push(std::make_unique<drake::lcmt_call_matlab>());
    message_queue_->back()->decode(rbuf->data, 0, rbuf->data_size);
  }

 private:
  std::queue<std::unique_ptr<drake::lcmt_call_matlab>>* message_queue_;
  std::mutex* message_mutex_;
};

}  // namespace

bool stop = false;

void signal_handler(int sig) {
  stop = true;
  mexPrintf("Ctrl-C pressed\n");
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  CheckIfLcmWillExplode();  // Run-time compatibility check.

  lcm::LCM lcm;
  if (!lcm.good()) mexErrMsgTxt("Bad LCM reference.");

  // TODO(russt): Take channel name as an optional input.
  std::string channel = "LCM_CALL_MATLAB";

  std::queue<std::unique_ptr<drake::lcmt_call_matlab>> message_queue;
  std::mutex message_mutex;

  Handler handler_object(&message_queue, &message_mutex);
  lcm.subscribe(channel, &Handler::HandleMessage, &handler_object);

  LCMLoop lcm_loop(&lcm);
  std::thread lcm_thread(&LCMLoop::LoopWithSelect, &lcm_loop);

  // Sets up signal handling for ctrl-C.
  struct sigaction act, oact;
  act.sa_handler = signal_handler;
  sigaction(SIGINT, &act, &oact);

  mexPrintf("Listening for messages on LCM_CALL_MATLAB...\n");

  FlushMatlabEventBuffer();

  std::map<int64_t, mxArray*> client_vars_;

  stop = false;
  while (!stop) {
    std::unique_ptr<drake::lcmt_call_matlab> msg(nullptr);

    {
      std::lock_guard<std::mutex> lock(message_mutex);
      if (!message_queue.empty()) {
        msg = std::move(message_queue.front());
        message_queue.pop();
      }
    }

    if (msg) {
      int i;
      std::vector<mxArray *> lhs(msg->nlhs), rhs(msg->nrhs);

      // Create the input arguments
      for (i = 0; i < msg->nrhs; i++) {
        if (msg->rhs[i].num_bytes == 0) {
          mexPrintf("rhs %d seems to have zero bytes.  dropping message %s.\n",
                    i, msg->function_name.c_str());
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
            rhs[i] = mxCreateDoubleMatrix(msg->rhs[i].rows, msg->rhs[i].cols,
                                          mxREAL);
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
            mxLogical* logical_data =
                static_cast<mxLogical*>(mxGetData(rhs[i]));
            for (int j = 0; j < static_cast<int>(msg->rhs[i].data.size());
                 j++) {
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
          static_cast<int>(lhs.size()), lhs.data(),
          static_cast<int>(rhs.size()), rhs.data(), msg->function_name.c_str());

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

      if (trapped_error) {
        break;
      }
    } else {
      // No messages this time, let matlab refresh.
      FlushMatlabEventBuffer();
    }
  }

  // Clean up remaining memory.
  for (const auto& iter : client_vars_) {
    mxDestroyArray(iter.second);
  }

  lcm_loop.Stop();
  lcm_thread.join();

  // Resets original signal handler.
  sigaction(SIGINT, &oact, &act);
}
