function testDoKinematics

% testTwistsVersusTdotAtlas('rpy');

regressionTestAtlas();

end

function testTwistsVersusTdotAtlas(floatingJointType)
robot = createAtlas(floatingJointType);

% robot = RigidBodyManipulator(fullfile('../../../examples/FurutaPendulum/FurutaPendulum.urdf'));

nq = robot.getNumStates() / 2; % TODO
nv = robot.getNumStates() / 2; % TODO

nBodies = length(robot.body);

nTests = 50;
for j = 1 : nTests
  q = zeros(nq, 1); %randn(nq, 1);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q, false, false, v);
  
  for i = 1 : nBodies
    TdotFromTwist = kinsol.T{i} * twistToTildeForm(kinsol.twist{i});
    valuecheck(TdotFromTwist, kinsol.Tdot{i}, 1e-12);
  end
end

end

function regressionTestAtlas()
replaceMatFile = false;

rng(23415, 'twister');
robot = createAtlas('rpy');

nq = robot.getNumStates() / 2;
nv = robot.getNumStates() / 2;

nTests = 5;
kinsols = cell(nTests, 1);
for i = 1 : nTests
  q = zeros(nq, 1); %randn(nq, 1);
  v = randn(nv, 1);
  kinsols{i} = robot.doKinematics(q, true, false, v);
end


filename = 'regressionTestAtlas.mat';
if replaceMatFile
  save(filename, varname(kinsols));
else
  data = load(filename);
  kinsolsFromMat = data.kinsols;
  for i = 1 : nTests
    if ~isequal(kinsols{i}.T, kinsolsFromMat{i}.T);
      error('kinsols not equal')
    end

    if ~isequal(kinsols{i}.Tdot, kinsolsFromMat{i}.Tdot);
      error('kinsols not equal')
    end
  
    if ~isequal(kinsols{i}.dTdq, kinsolsFromMat{i}.dTdq);
      error('kinsols not equal')
    end
    
    for index = 1 : length(length(robot.body))
      difference = max(max(abs(kinsols{i}.dTdqdot{index} - kinsolsFromMat{i}.dTdqdot{index})));
      if difference > 1e-12
        error('kinsols not equal')
      end
    end
  end
end

rng('default');
end

function robot = createAtlas(floatingJointType)
options.floating = floatingJointType;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
robot = RigidBodyManipulator(fullfile('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'),options);
warning(w);
end

function out = varname(~)
  out = inputname(1);
end