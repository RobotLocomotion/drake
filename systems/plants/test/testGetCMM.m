function testGetCMM()
floatingBaseParameterizations = {'rpy', 'quat'};

for parameterization = floatingBaseParameterizations
  robot = createAtlas(parameterization{:});
  testAtlasExternalWrenchVersusMassMatrix(robot);
  testTotalMomentumVersusTwistsTimesInertias(robot);
end

end

function testAtlasExternalWrenchVersusMassMatrix(robot)
% idea of test: compute a joint velocity vector corresponding for a case
% where there are only joint torques, but no external wrenches using H and
% C, then verify that the rate of change of centroidal momentum, obtained
% using the centroidal momentum matrix, is zero, i.e.
% vdot = inv(H) * (B * tau - C(v, q))
% hdot = A * vdot - Adot * v should be zero

robot = robot.setGravity(zeros(3, 1));
robot = compile(robot);
nq = robot.getNumPositions();
nv = robot.getNumVelocities();
nActuators = robot.getNumInputs();

nTests = 50;
for i = 1 : nTests
  q = randn(nq, 1);
  v = randn(nv, 1);
  tau = randn(nActuators, 1);
  
  kinsol = robot.doKinematics(q,true,false, v);
  [H, C, B] = robot.manipulatorDynamics(q, v, false);
  vdot = H \ (B * tau - C);
  
  [A, Adot_times_v] = getCMM(robot, kinsol);
  hdot = A * vdot + Adot_times_v; % rate of change of centroidal momentum
  valuecheck(zeros(6, 1), hdot, 1e-5);
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
  A = getCMM(robot, kinsol);
  h = A * v;
  
  inertias_world = computeInertiasInWorld(robot, kinsol);
  h_check = sum(cell2mat(cellfun(@mtimes, inertias_world, kinsol.twists, 'UniformOutput', false)), 2); % in world frame
  com = robot.getCOM(kinsol.q);
  transform_com_to_world = eye(4);
  transform_com_to_world(1:3, 4) = com;
  h_check = transformAdjoint(transform_com_to_world)' * h_check;
  
  valuecheck(h_check, h, 1e-10);
end
end