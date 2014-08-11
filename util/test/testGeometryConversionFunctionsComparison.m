function testGeometryConversionFunctionsComparison()

for i = 1 : 100
  q = uniformlyRandomQuat();
  rpy = uniformlyRandomRPY();
  [a2q, da2q, a2r, da2r, dda2r, r2a, q2a, dq2a, dq2R] = testGeometryConversionFunctionsmex(q, rpy);
  
  [a2q_check, da2q_check] = angularvel2quatdotMatrix(q);
  [a2r_check, da2r_check, dda2r_check] = angularvel2rpydotMatrix(rpy);
  r2a_check = rpydot2angularvelMatrix(rpy);
  [q2a_check, dq2a_check] = quatdot2angularvelMatrix(q);
  [~, dq2R_check] = quat2rotmat(q);
  
  valuecheck(a2q_check, a2q);
  valuecheck(da2q_check, da2q);
  valuecheck(a2r_check, a2r);
  valuecheck(da2r_check, da2r);
  valuecheck(reshape(dda2r_check, size(dda2r)), dda2r);
  valuecheck(r2a_check, r2a);
  valuecheck(q2a_check, q2a);
  valuecheck(dq2a_check, dq2a);
  valuecheck(dq2R_check, dq2R);
end

end