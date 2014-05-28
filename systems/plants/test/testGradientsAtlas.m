function testGradientsAtlas()
testFallingBrick('rpy');
testAtlas('rpy');

end

function testFallingBrick(floatingType)
options.floating = floatingType;
r = RigidBodyManipulator('FallingBrick.urdf',options);
testKinsolGradients(r, 'T', 'dTdq');
% testKinsolGradients(r, 'J', 'dJdq');
end

function testAtlas(floatingType)
r = createAtlas(floatingType);
testKinsolGradients(r, 'T', 'dTdq');
% testKinsolGradients(r, 'J', 'dJdq');
end

function testKinsolGradients(r, name, gradient_name)
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
    valuecheck(dXdq{j}(:, i), dXdqiNumerical{j}(:), 1e-5);
  end
end

end