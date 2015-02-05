function testdHomogTrans()
testFallingBrick('rpy');
testFallingBrick('quat');
end

function testFallingBrick(floatingType)
options.floating = floatingType;
m = RigidBodyManipulator('FallingBrick.urdf',options);
nq = m.getNumPositions();
q = getRandomConfiguration(m);
kinsol = doKinematics(m, q, false, false);
S = motionSubspace(m.body(2), q);
dHdqVec = dHomogTrans(kinsol.T{2}, S, jointQdot2v(m.getBody(2), q)); % special case where qdotToV for joint is qdotToV for whole robot...

delta = 1e-7;
for i = 1 : nq
  dq = zeros(nq, 1);
  dq(i) = delta;
  dkinsol = doKinematics(m, q + dq, false, false);
  dHdqNumerical = (dkinsol.T{2} - kinsol.T{2}) / delta;
  valuecheck(dHdqVec(:, i), dHdqNumerical(:), 1e-5);
end
end