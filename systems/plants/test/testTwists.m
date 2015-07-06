function testTwists()
testTwistsVersusTdotAtlas('rpy');
end

function testTwistsVersusTdotAtlas(floatingJointType)
robot = createAtlas(floatingJointType);

% robot = RigidBodyManipulator(fullfile('../../../examples/FurutaPendulum/FurutaPendulum.urdf'));

nq = robot.getNumPositions();
nv = robot.getNumVelocities();

nBodies = length(robot.body);

nTests = 50;
for j = 1 : nTests
  q = zeros(nq, 1); %randn(nq, 1);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q, false, false, v);
  twists = robot.twists(kinsol.T, q, v);
  
  for i = 1 : nBodies
    TdotFromTwist = kinsol.T{i} * twistToTildeForm(transformTwists(inv(kinsol.T{i}), twists{i}));
    valuecheck(TdotFromTwist, kinsol.Tdot{i}, 1e-12);
  end
end

end