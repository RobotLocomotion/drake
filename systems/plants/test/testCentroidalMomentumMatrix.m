function testCentroidalMomentumMatrix()
floatingBaseParameterizations = {'rpy', 'quat'};

for parameterization = floatingBaseParameterizations
  robot = createAtlas(parameterization{:});
  testTotalMomentumVersusTwistsTimesInertias(robot);
  testGradients(robot);
end

end

function testTotalMomentumVersusTwistsTimesInertias(robot)
% check that the centroidal momentum obtained from A * v is the same as the
% centroidal momentum obtained by computing the momentum (in world)of each
% body individually, summing, and transforming to COM frame.

nq = robot.getNumPositions();
nv = robot.getNumVelocities();

nTests = 50;
for i = 1 : nTests
  q = randn(nq, 1);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q,false,false, v);
  A = centroidalMomentumMatrix(robot, kinsol);
  h = A * v;
  
  inertias_world = inertiasInWorldFrame(robot, kinsol);
  h_check = sum(cell2mat(cellfun(@mtimes, inertias_world, kinsol.twists, 'UniformOutput', false)), 2); % in world frame
  com = robot.centerOfMass(kinsol.q);
  transform_com_to_world = eye(4);
  transform_com_to_world(1:3, 4) = com;
  h_check = transformAdjoint(transform_com_to_world)' * h_check;
  
  valuecheck(h_check, h, 1e-10);
end
end

function testGradients(robot)
nq = robot.getNumPositions();
nv = robot.getNumVelocities();

q = randn(nq, 1);
v = randn(nv, 1);
kinsol = robot.doKinematics(q,true,false, v, true);
[~, dA] = centroidalMomentumMatrix(robot, kinsol);

option.grad_method = {'taylorvar'};
[~, dA_geval] = geval(1, @(q) robot.centroidalMomentumMatrix(robot.doKinematics(q, false, false, v, true)), q, option);
valuecheck(dA_geval, dA, 1e-10);
end