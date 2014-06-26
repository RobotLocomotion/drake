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
  Tdot = computeTdots(kinsols{i}.T, kinsols{i}.twists);
  for i = 1 : nTests
    if ~isequal(kinsols{i}.T, kinsolsFromMat{i}.T);
      error('kinsols not equal')
    end

    for index = 1 : length(robot.body)
      if norm(Tdot{index} - Tdot{index}, Inf) > 1e-13
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
checkKinsolGradients(r, 'T', 'dTdq', 'q');
checkKinsolGradients(r, 'J', 'dJdq', 'q');
checkKinsolGradients(r, 'twists', 'dtwistsdq', 'q');
checkKinsolGradients(r, 'JdotV', 'dJdotVdq', 'q');
checkKinsolGradients(r, 'JdotV', 'dJdotVidv', 'v');
end

function testAtlas(floatingType)
r = createAtlas(floatingType);
checkKinsolGradients(r, 'T', 'dTdq', 'q');
checkKinsolGradients(r, 'J', 'dJdq', 'q');
checkKinsolGradients(r, 'twists', 'dtwistsdq', 'q');
checkKinsolGradients(r, 'JdotV', 'dJdotVdq', 'q');
checkKinsolGradients(r, 'JdotV', 'dJdotVidv', 'v');
end

function checkKinsolGradients(r, name, gradient_name, variable_name)
nq = r.getNumPositions();
nv = r.getNumVelocities();
nb = r.getNumBodies();

q = randn(nq, 1); %getRandomConfiguration(r);
v = randn(nv, 1);
kinsol = doKinematics(r, q, true, false, v, true);

X = kinsol.(name);
dXdvar = kinsol.(gradient_name);

delta = 1e-7;
if strcmp(variable_name, 'q')
  nvar = nq;
elseif strcmp(variable_name, 'v')
  nvar = nv;
else
  error('name not recognized');
end
  
for i = 1 : nvar
  if strcmp(variable_name, 'q')
    dq = zeros(nq, 1);
    dq(i) = delta;
    kinsol_delta = doKinematics(r, q + dq, false, false, v, true);
  elseif strcmp(variable_name, 'v')
    dv = zeros(nv, 1);
    dv(i) = delta;
    kinsol_delta = doKinematics(r, q, false, false, v + dv, true);
  else
    error('name not recognized');
  end
  X_delta = kinsol_delta.(name);
  dXdvariNumerical = cellfun(@(x, y) (x - y) / delta, X_delta, X, 'UniformOutput', false);

  for j = 2 : nb
    valuecheck(dXdvariNumerical{j}(:), dXdvar{j}(:, i), 1e-5);
  end
end

end

function out = varname(~)
  out = inputname(1);
end