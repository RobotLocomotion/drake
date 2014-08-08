function testCentroidalMomentumMatrixDotTimesV()
floatingBaseParameterizations = {'rpy', 'quat'};

for parameterization = floatingBaseParameterizations
  robot = createAtlas(parameterization{:});
  testAtlasExternalWrenchVersusMassMatrix(robot);
  testGradients(robot);
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
  
  kinsol = robot.doKinematics(q,false,false, v, true);
  [H, C, B] = robot.manipulatorDynamics(q, v, false);
  vdot = H \ (B * tau - C);
  
  A = centroidalMomentumMatrix(robot, kinsol);
  Adot_times_v = centroidalMomentumMatrixDotTimesV(robot, kinsol);
  hdot = A * vdot + Adot_times_v; % rate of change of centroidal momentum
  valuecheck(zeros(6, 1), hdot, 1e-5);
end
end

function testGradients(robot)
nq = robot.getNumPositions();
nv = robot.getNumVelocities();

q = randn(nq, 1);
v = randn(nv, 1);
kinsol = robot.doKinematics(q,true,false, v, true);
[~, dAdot_times_v] = centroidalMomentumMatrixDotTimesV(robot, kinsol);

option.grad_method = {'taylorvar'};
[~, dAdot_times_v_geval] = geval(1, @(q) robot.centroidalMomentumMatrixDotTimesV(robot.doKinematics(q, false, false, v, true)), q, option);
valuecheck(dAdot_times_v_geval, dAdot_times_v, 1e-10);
end