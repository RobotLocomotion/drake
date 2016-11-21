#include "drake/examples/Valkyrie/valkyrie_constants.h"

namespace drake {
namespace examples {
namespace valkyrie {

VectorX<double> RPYValkyrieFixedPointState() {
  VectorX<double> ret(kRPYValkyrieDoF * 2);
  ret << 0, 0, 1.025, 0, 0, 0, 0, 0, 0, 0, 0.300196631343025, 1.25, 0,
      0.785398163397448, 1.571, 0, 0, 0.300196631343025, -1.25, 0,
      -0.785398163397448, 1.571, 0, 0, 0, 0, -0.49, 1.205, -0.71, 0, 0, 0,
      -0.49, 1.205, -0.71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  return ret;
}

VectorX<double> RPYValkyrieFixedPointTorque() {
  VectorX<double> ff_torque(kRPYValkyrieDoF);
  ff_torque << 0, 0, 0, 0, 0, 0, 0, 54.07374714, -1.16973414,
      1.89429714, 3.778290679, -8.104844333, -1.370804286, 2.345797901,
      -0.3205054571, -0.2609708356, -0.1427544212, 3.778290679, 8.104844333,
      -1.370804286, -2.345797901, -0.3205054571, 0.2609708356, 0.1427544212,
      0.0009084321844, 12.02429585, -10.18358769, -118.6322523, 52.87796422,
      0.2418568986, 0.0009084320108, -11.43386868, -10.22606335, -116.9452938,
      52.24348208, 0.2418569007;
  return ff_torque;
}

}  // namespace valkyrie
}  // namespace examples
}  // namespace drake
