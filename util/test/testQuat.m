function testQuat
% Checks if the cpp and matlab computation are consistent for quatDiff,
% quatDiffInvAxis, and quatTransform

for i = 1:100
  q1 = randn(4,1);
  q2 = randn(4,1);
  axis = randn(3,1);
  u = randn(3,1);
  v = randn(3,1);
  q1 = q1/norm(q1);
  q2 = q2/norm(q2);
  axis = axis/norm(axis);
  u = u/norm(u);
  v = v/norm(v);
  [r,dr] = quatDiff(q1,q2);
  [e,de] = quatDiffAxisInvar(q1,q2,axis);
  [quat,dquat] = quatTransform(u,v);
  [r_mex,dr_mex,e_mex,de_mex,quat_mex,dquat_mex] = testQuatmex(q1,q2,axis,u,v);
  valuecheck(r,r_mex,1e-8);
  valuecheck(dr,dr_mex,1e-8);
  valuecheck(e,e_mex,1e-8);
  valuecheck(de,de_mex,1e-8);
  valuecheck(quat,quat_mex,1e-8);
  valuecheck(dquat,dquat_mex,1e-8);
end
end