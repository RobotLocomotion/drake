function testDHomogTrans()
ntests = 1000;
tmethod = 0;
tcheck = 0;
for i = 1 : ntests
  R = uniformlyRandomRotmat();
  p = randn(3, 1);
  T = [R, p; zeros(1, 3), 1];
  nq = 15;
  nv = 7;
  S = randn(6, nv);
  qdot_to_v = randn(nv, nq);
  
  tic;
  dT = dHomogTrans(T, S, qdot_to_v);
  tmethod = tmethod + toc;
  
  tic;
  dT_check = check(T, S, qdot_to_v);
  tcheck = tcheck + toc;
  
  valuecheck(dT_check, dT);
end

fprintf('time for dHomogTrans: %0.3f\n', tmethod);
fprintf('time for check: %0.3f\n', tcheck);

end

function ret = check(T, S, qdot_to_v)
% alternate method
% slower and currently doesn't work with TaylorVar
i = [7 10 3 9 2 5 13 14 15];
j = [1 1 2 2 3 3 4 5 6];
s = [1 -1 -1 1 1 -1 1 1 1];
L = sparse(i, j, s, 16, 6);
ret = kron(speye(4), sparse(T)) * L * S * qdot_to_v;
% S in world frame:
% ret = kron(H', eye(4)) * L * S * qdotToV;
end