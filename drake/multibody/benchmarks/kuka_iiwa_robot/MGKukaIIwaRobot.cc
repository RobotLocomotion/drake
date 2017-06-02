// File: MGKukaIIwaRobot.cpp created May 25 2017 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause
// MotionGenesis Professional Licensee: Paul Mitiguy.
//------------------------------------------------------------------------------

// GENERATED FILE DO NOT EDIT

#include "MGKukaIIwaRobot.h"

//------------------------------------------------------------------------------
namespace MotionGenesis {


//------------------------------------------------------------------------------
void  MGKukaIIwaRobot::SetVariablesFromArray( const double VAR[14] )
{
   qA = VAR[0];
   qB = VAR[1];
   qC = VAR[2];
   qD = VAR[3];
   qE = VAR[4];
   qF = VAR[5];
   qG = VAR[6];
   qAp = VAR[7];
   qBp = VAR[8];
   qCp = VAR[9];
   qDp = VAR[10];
   qEp = VAR[11];
   qFp = VAR[12];
   qGp = VAR[13];
}


//------------------------------------------------------------------------------
void  MGKukaIIwaRobot::CalculateOutput( )
{
   z[1] = cos(qA);
   z[2] = sin(qA);
   z[4] = cos(qB);
   z[5] = sin(qB);
   z[19] = cos(qC);
   z[20] = sin(qC);
   z[21] = z[5]*z[19];
   z[22] = z[5]*z[20];
   z[43] = cos(qD);
   z[44] = sin(qD);
   z[45] = z[4]*z[44] - z[21]*z[43];
   z[46] = z[20]*z[43];
   z[47] = z[4]*z[43] + z[21]*z[44];
   z[48] = z[20]*z[44];
   z[55] = z[19]*z[43];
   z[56] = z[22]*z[43];
   z[57] = z[19]*z[44];
   z[58] = z[22]*z[44];
   z[76] = 0.1845*z[19] + 0.42*z[55];
   z[77] = 0.1845*z[22] + 0.42*z[56];
   z[78] = 0.1845*z[45] - 0.42*z[21];
   z[79] = 0.1845*z[46] + 0.42*z[20];
   z[83] = cos(qE);
   z[84] = sin(qE);
   z[85] = -z[19]*z[84] - z[46]*z[83];
   z[86] = -z[22]*z[84] - z[45]*z[83];
   z[87] = z[44]*z[83];
   z[88] = z[44]*z[84];
   z[89] = z[45]*z[84] - z[22]*z[83];
   z[90] = z[46]*z[84] - z[19]*z[83];
   z[97] = z[79]*z[84] - z[76]*z[83];
   z[98] = z[78]*z[84] - z[77]*z[83];
   z[99] = z[77]*z[84] + z[78]*z[83];
   z[100] = z[76]*z[84] + z[79]*z[83];
   z[130] = z[97] + 0.2155*z[90];
   z[131] = z[98] + 0.2155*z[89];
   z[132] = z[99] - 0.2155*z[86];
   z[133] = z[100] - 0.2155*z[85];
   z[137] = cos(qF);
   z[138] = sin(qF);
   z[139] = z[43]*z[138] - z[87]*z[137];
   z[140] = z[47]*z[138] + z[86]*z[137];
   z[141] = z[85]*z[137] - z[48]*z[138];
   z[142] = z[84]*z[137];
   z[143] = z[43]*z[137] + z[87]*z[138];
   z[144] = z[47]*z[137] - z[86]*z[138];
   z[145] = -z[48]*z[137] - z[85]*z[138];
   z[146] = z[84]*z[138];
   z[153] = z[83]*z[137];
   z[154] = z[88]*z[137];
   z[155] = z[130]*z[137] - 0.42*z[57]*z[138];
   z[156] = z[131]*z[137] - 0.42*z[58]*z[138];
   z[157] = -z[130]*z[138] - 0.42*z[57]*z[137];
   z[158] = -z[131]*z[138] - 0.42*z[58]*z[137];
   z[159] = z[83]*z[138];
   z[160] = z[88]*z[138];
   z[186] = z[155] + 0.081*z[90];
   z[187] = z[156] + 0.081*z[89];
   z[188] = 0.081*z[83] + 0.4*z[153];
   z[189] = 0.081*z[88] + 0.4*z[154];
   z[190] = 0.081*z[139] - 0.4*z[87];
   z[191] = 0.081*z[140] - z[132];
   z[192] = 0.081*z[141] - z[133];
   z[193] = 0.081*z[142] + 0.4*z[84];
   z[197] = cos(qG);
   z[198] = sin(qG);
   z[209] = z[138]*z[198];
   z[210] = z[138]*z[197];
   z[221] = z[1]*z[4];
   z[222] = z[1]*z[5];
   z[223] = z[2]*z[4];
   z[224] = z[2]*z[5];
   z[225] = z[153]*z[197] - z[84]*z[198];
   z[226] = -z[84]*z[197] - z[153]*z[198];
   z[227] = -z[83]*z[198] - z[142]*z[197];
   z[228] = z[142]*z[198] - z[83]*z[197];
   z[229] = z[44]*z[222] + z[55]*z[221] - z[2]*z[46];
   z[230] = z[2]*z[48] + z[43]*z[222] - z[57]*z[221];
   z[231] = z[2]*z[19] + z[20]*z[221];
   z[232] = z[1]*z[46] + z[44]*z[224] + z[55]*z[223];
   z[233] = z[43]*z[224] - z[1]*z[48] - z[57]*z[223];
   z[234] = z[20]*z[223] - z[1]*z[19];
   z[235] = z[4]*z[44] - z[5]*z[55];
   z[236] = z[4]*z[43] + z[5]*z[57];
   z[237] = z[225]*z[229] + z[227]*z[231] - z[210]*z[230];
   z[238] = z[209]*z[230] + z[226]*z[229] + z[228]*z[231];
   z[239] = z[137]*z[230] + z[159]*z[229] - z[146]*z[231];
   z[240] = z[225]*z[232] + z[227]*z[234] - z[210]*z[233];
   z[241] = z[209]*z[233] + z[226]*z[232] + z[228]*z[234];
   z[242] = z[137]*z[233] + z[159]*z[232] - z[146]*z[234];
   z[243] = z[225]*z[235] - z[22]*z[227] - z[210]*z[236];
   z[244] = z[209]*z[236] + z[226]*z[235] - z[22]*z[228];
   z[245] = z[22]*z[146] + z[137]*z[236] + z[159]*z[235];
   z[250] = z[84]*z[231] - z[83]*z[229];
   z[251] = z[84]*z[234] - z[83]*z[232];
   z[252] = -z[22]*z[84] - z[83]*z[235];
   z[253] = z[83]*z[231] + z[84]*z[229];
   z[254] = z[83]*z[234] + z[84]*z[232];
   z[255] = z[84]*z[235] - z[22]*z[83];
   z[256] = z[137]*z[250] + z[138]*z[230];
   z[257] = z[137]*z[251] + z[138]*z[233];
   z[258] = z[137]*z[252] + z[138]*z[236];
   z[259] = z[137]*z[230] - z[138]*z[250];
   z[260] = z[137]*z[233] - z[138]*z[251];
   z[261] = z[137]*z[236] - z[138]*z[252];


   R_NG[0][0] = z[237];
   R_NG[0][1] = z[238];
   R_NG[0][2] = z[239];
   R_NG[1][0] = z[240];
   R_NG[1][1] = z[241];
   R_NG[1][2] = z[242];
   R_NG[2][0] = z[243];
   R_NG[2][1] = z[244];
   R_NG[2][2] = z[245];

   p_NoGo_N[0] = 0.081*z[259] + 0.4*z[230] + 0.42*z[222];
   p_NoGo_N[1] = 0.081*z[260] + 0.4*z[233] + 0.42*z[224];
   p_NoGo_N[2] = 0.36 + 0.081*z[261] + 0.4*z[236] + 0.42*z[4];

   w_NG_N[0] = z[256]*(z[138]*qEp+z[139]*qCp+z[140]*qAp+z[141]*qBp+z[142]*qDp) + z[259]*(qGp+z[137]*qEp+z[143]*qCp+
       z[144]*qAp+z[145]*qBp-z[146]*qDp) - z[253]*(qFp-z[83]*qDp-z[88]*qCp-z[89]*qAp-z[90]*qBp);
   w_NG_N[1] = z[257]*(z[138]*qEp+z[139]*qCp+z[140]*qAp+z[141]*qBp+z[142]*qDp) + z[260]*(qGp+z[137]*qEp+z[143]*qCp+
       z[144]*qAp+z[145]*qBp-z[146]*qDp) - z[254]*(qFp-z[83]*qDp-z[88]*qCp-z[89]*qAp-z[90]*qBp);
   w_NG_N[2] = z[258]*(z[138]*qEp+z[139]*qCp+z[140]*qAp+z[141]*qBp+z[142]*qDp) + z[261]*(qGp+z[137]*qEp+z[143]*qCp+
       z[144]*qAp+z[145]*qBp-z[146]*qDp) - z[255]*(qFp-z[83]*qDp-z[88]*qCp-z[89]*qAp-z[90]*qBp);

   v_NGo_N[0] = 0.4*z[259]*(2.5*z[157]*qBp+2.5*z[158]*qAp-z[159]*qDp-z[160]*qCp) - 0.081*z[256]*(qFp-12.34567901234568*
       z[186]*qBp-12.34567901234568*z[187]*qAp-12.34567901234568*z[188]*qDp-12.34567901234568*z[189]*qCp) - 0.081*
       z[253]*(z[138]*qEp+12.34567901234568*z[190]*qCp+12.34567901234568*z[191]*qAp+12.34567901234568*z[192]*qBp+
       12.34567901234568*z[193]*qDp);
   v_NGo_N[1] = 0.4*z[260]*(2.5*z[157]*qBp+2.5*z[158]*qAp-z[159]*qDp-z[160]*qCp) - 0.081*z[257]*(qFp-12.34567901234568*
       z[186]*qBp-12.34567901234568*z[187]*qAp-12.34567901234568*z[188]*qDp-12.34567901234568*z[189]*qCp) - 0.081*
       z[254]*(z[138]*qEp+12.34567901234568*z[190]*qCp+12.34567901234568*z[191]*qAp+12.34567901234568*z[192]*qBp+
       12.34567901234568*z[193]*qDp);
   v_NGo_N[2] = 0.4*z[261]*(2.5*z[157]*qBp+2.5*z[158]*qAp-z[159]*qDp-z[160]*qCp) - 0.081*z[258]*(qFp-12.34567901234568*
       z[186]*qBp-12.34567901234568*z[187]*qAp-12.34567901234568*z[188]*qDp-12.34567901234568*z[189]*qCp) - 0.081*
       z[255]*(z[138]*qEp+12.34567901234568*z[190]*qCp+12.34567901234568*z[191]*qAp+12.34567901234568*z[192]*qBp+
       12.34567901234568*z[193]*qDp);
}


//------------------------------------------------------------------------------
}       // End of namespace MotionGenesis.


