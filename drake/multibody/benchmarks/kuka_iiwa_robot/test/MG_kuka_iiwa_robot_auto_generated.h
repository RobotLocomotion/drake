// GENERATED FILE DO NOT EDIT·
//------------------------------------------------------------------------------
// File: MGKukaIIwaRobot.cc created Jul 31 2017 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause
// MotionGenesis Professional Licensee: Paul Mitiguy.
//------------------------------------------------------------------------------
#include <cmath>

//------------------------------------------------------------------------------
#ifndef MGKUKAIIWAROBOTAUTOGENERATED_
#define MGKUKAIIWAROBOTAUTOGENERATED_

//------------------------------------------------------------------------------
namespace MotionGenesis {


//------------------------------------------------------------------------------
class MGKukaIIwaRobotAutoGenerated {
 public:
  MGKukaIIwaRobotAutoGenerated() {}

  // Set member state variables from array.
  void  SetVariablesFromArray(const double VAR[21]);

  // Calculate quantities listed in MotionGenesis Output statements.
  void  CalculateOutput();

  // Class data.
  double qA, qB, qC, qD, qE, qF, qG;
  double qAp, qBp, qCp, qDp, qEp, qFp, qGp;
  double qApp, qBpp, qCpp, qDpp, qEpp, qFpp, qGpp;
  double fAx, fAy, fAz, fBx, fBy, fBz, fCx, fCy, fCz, fDx, fDy, fDz, fEx, fEy,
         fEz, fFx, fFy, fFz, fGx, fGy, fGz, tAx, tAy, tAz, tBx, tBy, tBz, tCx,
         tCy, tCz, tDx, tDy, tDz, tEx, tEy, tEz, tFx, tFy, tFz, tGx, tGy, tGz;
  double z[221], R_NG[3][3], p_NoGo_N[3], w_NG_N[3], v_NGo_N[3],
         alpha_NG_N[3], a_NGo_N[3], fA[3], tA[3], fB[3], tB[3],
         fC[3], tC[3], fD[3], tD[3], fE[3], tE[3], fF[3], tF[3], fG[3], tG[3];
};


//------------------------------------------------------------------------------
}       // End of namespace MotionGenesis.

//------------------------------------------------------------------------------
#endif  //  MGKUKAIIWAROBOTAUTOGENERATED_

