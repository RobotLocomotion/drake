function testdHdq()
testFallingBrick('rpy');
testFallingBrick('quat');
end

function testFallingBrick(floatingType)
options.floating = floatingType;
m = RigidBodyManipulator('FallingBrick.urdf',options);
nq = m.getNumPositions();
q = getRandomConfiguration(m);
kinsol = doKinematics(m, q, false, false);
dHdqVec = dHdq(kinsol.T{2}, kinsol.J{2}, kinsol.qdotToV); % special case where qdotToV for joint is qdotToV for whole robot...

delta = 1e-7;
for i = 1 : nq
  dq = zeros(nq, 1);
  dq(i) = delta;
  dkinsol = doKinematics(m, q + dq, false, false);
  dHdqNumerical = (dkinsol.T{2} - kinsol.T{2}) / delta;
  valuecheck(dHdqVec(:, i), dHdqNumerical(:), 1e-5);
end
end