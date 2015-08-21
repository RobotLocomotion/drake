#include "LCMCoordinateFrame.h"
#include <lcmtypes/lcmt_drake_signal.hpp>

using namespace std;
using namespace Eigen;

static bool encode(const CoordinateFrame& frame, const double t, const VectorXd& x, lcmt_drake_signal& msg) {
  msg.timestamp = static_cast<int64_t>(t*1000);
  msg.dim = frame.getDim();
  msg.coord = frame.getCoordinateNames(); // note: inefficient to do a deep copy every time
  for (int i=0; i<msg.dim; i++) msg.val[i] = x(i);
}

static bool decode(const CoordinateFrame& frame, const lcmt_drake_signal& msg, double& t, VectorXd& x) {
  throw runtime_error("decode lcmt_drake_signal not implemented yet (will be trivial).");
}


template DLLEXPORT LCMCoordinateFrame<lcmt_drake_signal>;

