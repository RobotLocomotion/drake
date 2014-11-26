function testDoKinematics

regressionTestAtlas();

testFloatingObject();
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

    for index = 1 : length(robot.body)
      if norm(kinsols{i}.Tdot{index} - kinsolsFromMat{i}.Tdot{index}, Inf) > 1e-13
        error('kinsols not equal')
      end
    end
  
    if ~isequal(kinsols{i}.dTdq, kinsolsFromMat{i}.dTdq);
      error('kinsols not equal')
    end
    
    for index = 1 : length(robot.body)
      if norm(kinsols{i}.dTdqdot{index} - kinsolsFromMat{i}.dTdqdot{index}, Inf) > 1e-12
        error('kinsols not equal')
      end
    end
  end
end

rng('default');
end

function out = varname(~)
  out = inputname(1);
end

function testFloatingObject()
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
robot = robot.addRobotFromURDF([getDrakePath,'/solvers/trajectoryOptimization/dev/block.urdf'],[],[],struct('floating',true));
nq = robot.getNumPositions();
q = randn(nq,1);
kinsol = robot.doKinematics(q,false,false);
end