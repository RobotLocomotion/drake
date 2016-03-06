function testQuat
% Checks if the cpp and matlab computation are consistent for quatDiff,
% quatDiffInvAxis, quatProduct, and quatRotateVector

if ~exist('testQuatmex','file')
  error('Drake:MissingDependency:testQuatmex','Skipping this test because I can''t find testQuatmex (presumably because you didn''t have the prequisites at compile time');
end

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
  [q3,dq3] = quatProduct(q1,q2);
  [w,dw] = quatRotateVec(q1,u);
  [r_mex,dr_mex,e_mex,de_mex,q3_mex,dq3_mex,w_mex,dw_mex] = testQuatmex(q1,q2,axis,u,v);
  valuecheck(r,r_mex,1e-8);
  valuecheck(dr,dr_mex,1e-8);
  valuecheck(e,e_mex,1e-8);
  valuecheck(de,de_mex,1e-8);
  valuecheck(q3,q3_mex,1e-8);
  valuecheck(dq3,dq3_mex,1e-8);
  valuecheck(w,w_mex,1e-8);
  valuecheck(dw,dw_mex,1e-8);
end
end
