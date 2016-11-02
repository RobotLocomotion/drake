function testGeometricJacobianDotV()
testAtlas('rpy');
% testAtlas('quat'); % TODO: generating robot currently results in an error.

end

function testAtlas(floatingJointType)
robot = createAtlas(floatingJointType);
testVersusNumericalDifferentiation(robot);

end

function testVersusNumericalDifferentiation(robot)
dt = 1e-8;

nv = robot.getNumVelocities();

nTests = 50;
nBodies = length(robot.body);
bodyRange = [1, nBodies];
testNumber = 1;
while testNumber <= nTests
  endEffector = randi(bodyRange);
  base = randi(bodyRange);
  expressedIn = randi(bodyRange);
  if base ~= endEffector
    % random state
    q = getRandomConfiguration(robot);
    v = randn(nv, 1);
    
    % compute Jacobian, JDotV
    kinsol = robot.doKinematics(q, false, false, v);
    kinsol.twists = robot.twists(kinsol.T, q, v);
    [J0, vIndices] = robot.geometricJacobian(kinsol, base, endEffector, expressedIn);
    JDotV = robot.geometricJacobianDotV(kinsol, base, endEffector, expressedIn);
    
    % integrate
    q = q + v * dt;
    
    % update kinematics, Jacobian
    kinsolNew = robot.doKinematics(q, false, false, v);
    J1 = robot.geometricJacobian(kinsolNew, base, endEffector, expressedIn);
    
    % compute JDotV through numerical differentiation
    JDotVNumerical = (J1 - J0) / dt * v(vIndices);
    
%     disp(testNumber);
    % check
    [same, errstr] = valuecheck(JDotV, JDotVNumerical, 1e-5);
    
    if ~same
      fprintf('base: %d\n', base);
      fprintf('endEffector: %d\n', endEffector);
      fprintf('expressedIn: %d\n', expressedIn);
      error(errstr);
    end
    
    testNumber = testNumber + 1;
  end
end
end