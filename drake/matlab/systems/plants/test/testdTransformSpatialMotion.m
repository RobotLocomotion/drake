function testdTransformSpatialMotion()
testFallingBrick('rpy');
testFallingBrick('quat');
end

function testFallingBrick(floatingType,options)
options.floating = floatingType;
m = RigidBodyManipulator('FallingBrick.urdf',options);
nq = m.getNumPositions();
q = getRandomConfiguration(m);
kinsol = doKinematics(m, q, false, false);

H = kinsol.T{2};
S = motionSubspace(m.body(2), q);
qdotToV = jointQdot2v(m.getBody(2), q);
dHdqVec = dHomogTrans(H, S, qdotToV);
X = randn(6, 4);
dXdq = zeros(numel(X), nq);
dAdH_times_X_1 = dTransformSpatialMotion(H, X, dHdqVec, dXdq);

delta = 1e-7;
for i = 1 : nq
  dq = zeros(nq, 1);
  dq(i) = delta;
  dkinsol = doKinematics(m, q + dq, false, false);
  adH_times_x_1 = transformAdjoint(kinsol.T{2}) * X;
  adH_times_x_2 = transformAdjoint(dkinsol.T{2}) * X;
  dAdH_times_X_dqiNumerical = (adH_times_x_2 - adH_times_x_1) / delta;
  valuecheck(dAdH_times_X_1(:, i), dAdH_times_X_dqiNumerical(:), 1e-5);
end
end