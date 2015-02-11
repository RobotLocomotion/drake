function benchmarkRigidBodyManipulator
%NOTEST

robot = createAtlas('rpy');

kinsol_options.use_mex = true;
include_gradients = false;

kinsol_options.compute_gradients = include_gradients;
iters = 100;

sides = {'l', 'r'};
hands = zeros(length(sides), 1);
feet = zeros(length(sides), 1);
for i = 1 : length(sides)
  hands(i) = robot.findLinkId([sides{i} '_hand']);
  feet(i) = robot.findLinkId([sides{i} '_foot']);
end
head = robot.findLinkId('head');

npoints_feet = 4;
npoints_hands = 1;
npoints_head = 1;

if include_gradients
  rotation_type = 0;
else
  rotation_type = 1;
end

tic
for i = 1 : iters
  q = randn(robot.getNumPositions(), 1);
  kinsol = robot.doKinematics(q, [], [], [], kinsol_options);
  
%   if robot.use_new_kinsol
%     robot.getCMMdA(kinsol); % getCMMdA is mexed, getCMM is not
%   else
%     robot.getCMM(kinsol); % getCMM is mexed, getCMMdA is not
%   end
  
  for j = 1 : length(feet)
    points = randn(3, npoints_feet);
    if include_gradients
      [~, ~, ~] = robot.forwardKin(kinsol, feet(j), points, rotation_type);
    else
      [~, ~] = robot.forwardKin(kinsol, feet(j), points, rotation_type);
    end
  end
  for j = 1 : length(hands)
    points = randn(3, npoints_hands);
    if include_gradients
      [~, ~, ~] = robot.forwardKin(kinsol, hands(j), points, rotation_type);
    else
      [~, ~] = robot.forwardKin(kinsol, hands(j), points, rotation_type);
    end
  end
  points = randn(3, npoints_head);
  if include_gradients
    [~, ~, ~] = robot.forwardKin(kinsol, head, points, rotation_type);
  else
    [~, ~] = robot.forwardKin(kinsol, head, points, rotation_type);
  end
end
time = toc;
fprintf('time elapsed per iteration: %0.8f\n', time / iters);
end