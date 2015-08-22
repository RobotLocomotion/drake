#include "LCMCoordinateFrame.h"
#include <thread>
#include "timeUtil.h"

using namespace std;
using namespace Eigen;

bool encode(const CoordinateFrame* frame, const double t, const VectorXd& x, drake::lcmt_drake_signal& msg) {
  msg.timestamp = static_cast<int64_t>(t*1000);
  msg.dim = frame->getDim();
  msg.coord = frame->getCoordinateNames(); // note: inefficient to do a deep copy every time
  for (int i=0; i<msg.dim; i++) msg.val.push_back(x(i));
  return true;
}

bool decode(const CoordinateFrame* frame, const drake::lcmt_drake_signal& msg, double& t, VectorXd& x) {
  t = double(msg.timestamp)/1000.0;
  for (int i=0; i<msg.dim; i++) { x(i) = msg.val[i]; }  // todo: make this more robust by doing string matching on the coordinate names (with a hashmap?)
  return true;
}


class LCMHandler {

public:

  thread ThreadHandle;
  shared_ptr<lcm::LCM> LCMHandle;
  bool ShouldStop;

  LCMHandler() : ShouldStop(false) {}

  bool WaitForLCM(double timeout) {
    int lcmFd = this->LCMHandle->getFileno();

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout * 1e6;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcmFd, &fds);

    int status = select(lcmFd + 1, &fds, 0, 0, &tv);
    if (status == -1 && errno != EINTR)
    {
      cout << "select() returned error: " << errno << endl;
    }
    else if (status == -1 && errno == EINTR)
    {
      cout << "select() interrupted" << endl;
    }
    return (status > 0 && FD_ISSET(lcmFd, &fds));
  }

  void ThreadLoopWithSelect() {
    cout << "ThreadLoopWithSelect " << this_thread::get_id() << endl;

    while (!this->ShouldStop)
    {
      const double timeoutInSeconds = 0.3;
      bool lcmReady = this->WaitForLCM(timeoutInSeconds);

      if (this->ShouldStop) break;

      if (lcmReady) {
        if (this->LCMHandle->handle() != 0) {
          cout << "lcm->handle() returned non-zero" << endl;
          break;
        }
      }
    }

    cout << "ThreadLoopWithSelect ended " << this_thread::get_id() << endl;
  }

  void ThreadLoop() {
    while (!this->ShouldStop) {
      if (this->LCMHandle->handle() != 0) {
        cout << "lcm->handle() returned non-zero" << endl;
        break;
      }
    }
  }

  bool IsRunning() {
    return this->ThreadHandle.joinable();
  }

  void Start() {
    cout << "LCMHandler start... " << this_thread::get_id() << endl;
    if (this->IsRunning()) {
      cout << "already running lcm thread. " << this_thread::get_id() << endl;
      return;
    }

    this->ShouldStop = false;
    this->ThreadHandle = std::thread(&LCMHandler::ThreadLoopWithSelect, this);
  }

  void Stop() {
    this->ShouldStop = true;
    this->ThreadHandle.join();
  }

};


void runLCM(const shared_ptr<lcm::LCM>& lcm, const DrakeSystemPtr& sys, double t0, double tf, const Eigen::VectorXd& x0, DrakeSystem::SimulationOptions* options) {
  if(!lcm->good())
    throw runtime_error("bad LCM reference");

  DrakeSystemPtr lcm_sys = sys->output_frame->setupLCMOutputs(sys);

  // set up lcm inputs
  bool has_lcm_input = false;
  {
    DrakeSystemPtr new_sys = sys->input_frame->setupLCMInputs(lcm_sys);
    has_lcm_input = (new_sys != lcm_sys);
    lcm_sys = new_sys;
  }

  if (has_lcm_input && sys->state_frame->getDim()<1 && lcm_sys->isTimeInvariant()) {
    // then this is really a static function, not a dynamical system.
    // block on receiving lcm input and process the output exactly when a new input message is received.
    // note: this will never return (unless there is an lcm error)

    double t = 0.0;
    VectorXd x = VectorXd::Zero(0);
    VectorXd u = VectorXd::Zero(lcm_sys->input_frame->getDim());

    while(1) {
      if (lcm->handle() != 0) { throw runtime_error("something went wrong in lcm.handle"); }
      lcm_sys->output(t,x,u);
    }
  } else {
    // then the system's dynamics should control when the output is produced.  try to run a simulation in real time

    if (!options) options = &(sys->default_simulation_options);
    DrakeSystem::SimulationOptions lcm_options = *options;
    lcm_options.realtime_factor = 1.0;

    if (has_lcm_input) {
      throw runtime_error("not implemented yet");
      // todo: spawn a thread for the lcm handle loop
    }

    lcm_sys->simulate(t0,tf,x0,lcm_options);

    if (has_lcm_input) {
      // todo: shutdown the lcm handle thread
    }
  }
}
