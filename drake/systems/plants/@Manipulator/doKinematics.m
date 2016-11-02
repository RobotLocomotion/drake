function kinsol = doKinematics(q, v)
kinsol.q = q;
if nargin > 1
  kinsol.v = v;
end
kinsol.mex = false;
end

