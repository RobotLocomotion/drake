function testMotionSubspace()
options.floating = 'rpy';
r = RigidBodyManipulator('FallingBrick.urdf',options);
q = getRandomConfiguration(r);
nq = length(q);
[S, dSdq] = motionSubspace(r.body(2), q);

% numerically differentiate
delta = 1e-7;
for i = 2 : nq
  dq = zeros(nq, 1);
  dq(i) = delta;
  S_delta = motionSubspace(r.body(2), q + dq);
  dSdqiNumerical = (S_delta - S) / delta;
  valuecheck(dSdqiNumerical(:), dSdq(:, i), 1e-5);
end

end