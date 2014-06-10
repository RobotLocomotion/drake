function testDoKinematics

regressionTestAtlas();

testFallingBrick('rpy');
testFallingBrick('quat');
testAtlas('rpy');
testAtlas('quat');

end


function regressionTestAtlas()
replaceMatFile = false;

rng(23415, 'twister');
robot = createAtlas('rpy');

nq = robot.getNumPositions();
nv = robot.getNumVelocities();

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

    for index = 1 : length(robot.body)
      if norm(kinsols{i}.Tdot{index} - kinsolsFromMat{i}.Tdot{index}, Inf) > 1e-13
        error('kinsols not equal')
      end
    end
  end
end

rng('default');
end

function testFallingBrick(floatingType)
options.floating = floatingType;
r = RigidBodyManipulator('FallingBrick.urdf',options);
checkKinsolGradients(r, 'T', 'dTdq');
checkKinsolGradients(r, 'J', 'dJdq');
checkKinsolGradients(r, 'twists', 'dtwistsdq');
checkKinsolGradients(r, 'JdotV', 'dJdotVdq');
end

function testAtlas(floatingType)
r = createAtlas(floatingType);
checkKinsolGradients(r, 'T', 'dTdq');
checkKinsolGradients(r, 'J', 'dJdq');
checkKinsolGradients(r, 'twists', 'dtwistsdq');
checkKinsolGradients(r, 'JdotV', 'dJdotVdq');
end

function checkKinsolGradients(r, name, gradient_name)
nq = r.getNumPositions();
nv = r.getNumVelocities();
nb = r.getNumBodies();

q = randn(nq, 1); %getRandomConfiguration(r);
v = randn(nv, 1);
kinsol = doKinematics(r, q, true, false, v, true);

X = kinsol.(name);
dXdq = kinsol.(gradient_name);

delta = 1e-7;
for i = 1 : nq
  dq = zeros(nq, 1);
  dq(i) = delta;
  kinsol_delta = doKinematics(r, q + dq, false, false, v, true);
  X_delta = kinsol_delta.(name);
  dXdqiNumerical = cellfun(@(x, y) (x - y) / delta, X_delta, X, 'UniformOutput', false);
  
  for j = 2 : nb
    valuecheck(dXdqiNumerical{j}(:), dXdq{j}(:, i), 1e-5);
  end
end

end

function out = varname(~)
  out = inputname(1);
end