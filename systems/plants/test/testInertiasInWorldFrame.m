function testInertiasInWorldFrame()
testAtlas('rpy');
testAtlas('quat');
end

function testAtlas(floatingJointType)
robot = createAtlas(floatingJointType);
checkInertiaGradients(robot);
end

function checkInertiaGradients(robot)
q = getRandomConfiguration(robot);
kinsol = robot.doKinematics(q, true, false);
[~, dinertias_world] = robot.inertiasInWorldFrame(kinsol);

option.grad_method = 'taylorvar';

for i = 2 : robot.getNumBodies()
  [~, dinertia_world_geval] = geval(1, @(q) computeInertias(robot, q, i), q, option);
  valuecheck(dinertias_world{i}, dinertia_world_geval, 1e-12);
end
end

function inertia_world = computeInertias(robot, q, i)
kinsol = robot.doKinematics(q, false, false);
inertias_world = robot.inertiasInWorldFrame(kinsol);
inertia_world = inertias_world{i};
end