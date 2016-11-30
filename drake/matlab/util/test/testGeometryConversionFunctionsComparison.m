function testGeometryConversionFunctionsComparison()

if exist('testGeometryConversionFunctionsmex','file')~=3
  error('Drake:MissingDependency', 'Cannot find testGeometryConversionFunctionsmex. It may not have been compiled due to a missing dependency.');
end

nq = 15;

for i = 1 : 100
  q = rpy2quat(uniformlyRandomNonsingularRPY());
  dq = randn(numel(q), nq);
  [rpy_mex,omega2qd, domega2qd, omega2rpyd, domega2rpyd, ddomega2rpyd, rpyd2omega, drpyd2omega, qd2omega, dqd2omega, dq2R, drpydR, dqdR] = testGeometryConversionFunctionsmex(q, dq);
  
  rpy = quat2rpy(q);
  [a2q_check, da2q_check] = angularvel2quatdotMatrix(q);
  [a2r_check, da2r_check, dda2r_check] = angularvel2rpydotMatrix(rpy);
  [r2a_check,dr2a_check] = rpydot2angularvelMatrix(rpy);
  [q2a_check, dq2a_check] = quatdot2angularvelMatrix(q);
  R = quat2rotmat(q);
  [~, dq2R_check] = quat2rotmat(q);
  dR = dq2R_check * dq;
  [~, drpydR_check] = rotmat2rpy(R, dR);
  [~, dqdR_check] = rotmat2quat(R, dR);
  
  valuecheck(rpy, rpy_mex);
  valuecheck(a2q_check, omega2qd);
  valuecheck(da2q_check, domega2qd);
  valuecheck(a2r_check, omega2rpyd);
  valuecheck(da2r_check, domega2rpyd);
  valuecheck(reshape(dda2r_check, size(ddomega2rpyd)), ddomega2rpyd);
  valuecheck(r2a_check, rpyd2omega);
  valuecheck(dr2a_check,drpyd2omega);
  valuecheck(q2a_check, qd2omega);
  valuecheck(dq2a_check, dqd2omega);
  valuecheck(dq2R_check, dq2R);
  valuecheck(drpydR_check, drpydR);
  valuecheck(dqdR_check, dqdR);
end

end
