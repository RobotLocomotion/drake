function testRigidBodyManipulatorGradients()
testFallingBrick('rpy');
testFallingBrick('quat');
testAtlas('rpy');
testAtlas('quat');
end

function testFallingBrick(floatingType)
options.floating = floatingType;
r = RigidBodyManipulator('FallingBrick.urdf',options);
checkKinsolGradients(r, 'T', 'dTdq');
checkKinsolGradients(r, 'J', 'dJdq');
end

function testAtlas(floatingType)
r = createAtlas(floatingType);
checkKinsolGradients(r, 'T', 'dTdq');
checkKinsolGradients(r, 'J', 'dJdq');
end

function checkKinsolGradients(r, name, gradient_name)
nq = r.getNumPositions();
nb = r.getNumBodies();

q = getRandomConfiguration(r);
kinsol = doKinematics(r, q, true, false);

X = kinsol.(name);
dXdq = kinsol.(gradient_name);

delta = 1e-7;
for i = 1 : nq
  dq = zeros(nq, 1);
  dq(i) = delta;
  kinsol_delta = doKinematics(r, q + dq, false, false);
  X_delta = kinsol_delta.(name);
  dXdqiNumerical = cellfun(@(x, y) (x - y) / delta, X_delta, X, 'UniformOutput', false);
  
  for j = 2 : nb
    valuecheck(dXdqiNumerical{j}(:), dXdq{j}(:, i), 1e-5);
  end
end

end