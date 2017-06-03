// File: MGKukaIIwaRobot.h created May 25 2017 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause
// MotionGenesis Professional Licensee: Paul Mitiguy.
//------------------------------------------------------------------------------
// GENERATED FILE DO NOT EDIT·
#include <cmath>

//------------------------------------------------------------------------------
#ifndef MGKUKAIIWAROBOT_
#define MGKUKAIIWAROBOT_

//------------------------------------------------------------------------------
namespace MotionGenesis {


//------------------------------------------------------------------------------
class MGKukaIIwaRobot {
 public:
  MGKukaIIwaRobot() {}

  // Set member state variables from array.
  void  SetVariablesFromArray(const double VAR[14]);

  // Calculate quantities listed in MotionGenesis Output statements.
  void  CalculateOutput();

  // Class data.
  double qA, qB, qC, qD, qE, qF, qG, qAp, qBp, qCp, qDp, qEp, qFp, qGp;
  double z[262], R_NG[3][3], p_NoGo_N[3], w_NG_N[3], v_NGo_N[3];
};


//------------------------------------------------------------------------------
}       // End of namespace MotionGenesis.

//------------------------------------------------------------------------------
#endif  //  MGKUKAIIWAROBOT_

