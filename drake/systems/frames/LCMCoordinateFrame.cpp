#include "LCMCoordinateFrame.h"
#include <thread>

using namespace std;
using namespace Eigen;

bool encode(const CoordinateFrame& frame, const double t, const VectorXd& x, drake::lcmt_drake_signal& msg) {
  msg.timestamp = static_cast<int64_t>(t*1000);
  msg.dim = frame.getDim();
  msg.coord = frame.getCoordinateNames(); // note: inefficient to do a deep copy every time
  for (int i=0; i<msg.dim; i++) msg.val.push_back(x(i));
  return true;
}

bool decode(const CoordinateFrame& frame, const drake::lcmt_drake_signal& msg, double& t, VectorXd& x) {
  t = double(msg.timestamp)/1000.0;
  for (int i=0; i<msg.dim; i++) { x(i) = msg.val[i]; }  // todo: make this more robust by doing string matching on the coordinate names (with a hashmap?)
  return true;
}

#if defined(_WIN32) || defined(_WIN64)
#include <Winsock2.h>
#else
#include <sys/select.h>
#endif

bool waitForLCM(lcm::LCM& lcm, double timeout) {
  int lcmFd = lcm.getFileno();

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

class LCMLoop {
public:
  bool stop;
  lcm::LCM& lcm;

  LCMLoop(lcm::LCM& _lcm) : lcm(_lcm), stop(false) {}

  void loopWithSelect() {
//    cout << "starting lcm handler thread " << this_thread::get_id() << endl;

    while (!this->stop)
    {
      const double timeoutInSeconds = 0.3;
      bool lcmReady = waitForLCM(lcm,timeoutInSeconds);

      if (this->stop) break;

      if (lcmReady) {
        if (lcm.handle() != 0) {
          cout << "lcm->handle() returned non-zero" << endl;
          break;
        }
      }
    }

//    cout << "stopping lcm handler thread " << this_thread::get_id() << endl;
  }
};


void runLCM(const DrakeSystemPtr& sys, lcm::LCM& lcm, double t0, double tf, const Eigen::VectorXd& x0, const DrakeSystem::SimulationOptions* options) {
  if(!lcm.good())
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
      if (lcm.handle() != 0) { throw runtime_error("something went wrong in lcm.handle"); }
      lcm_sys->output(t,x,u);
    }
  } else {
    // then the system's dynamics should control when the output is produced.  try to run a simulation in real time

    if (!options) options = &(sys->default_simulation_options);
    DrakeSystem::SimulationOptions lcm_options = *options;
    lcm_options.realtime_factor = 1.0;

    LCMLoop lcm_loop(lcm);
    thread lcm_thread;
    if (has_lcm_input) {
      // only start up the listener thread if I actually have inputs to listen to
      lcm_thread = thread(&LCMLoop::loopWithSelect, &lcm_loop);
    }

    lcm_sys->simulate(t0,tf,x0,lcm_options);

    if (has_lcm_input) {
      // shutdown the lcm thread
      lcm_loop.stop = true;
      lcm_thread.join();
    }
  }
}
