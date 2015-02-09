function benchmarkRigidBodyManipulator
%NOTEST
robot = createAtlas('rpy');

iters = 1000;

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
kinsol_options.use_mex = true;

for scenario = [1 2]
  % 1: CMM + a couple of Jacobians
  % 2: manipulatorDynamics
  for include_gradients = [true false];
    kinsol_options.compute_gradients = include_gradients;

    if include_gradients
      rotation_type = 0;
    else
      rotation_type = 1;
    end
    
    tic
    for i = 1 : iters
      q = randn(robot.getNumPositions(), 1);
      
      if scenario == 1
        kinsol = robot.doKinematics(q, [], kinsol_options);
        if robot.use_new_kinsol
          robot.getCMMdA(kinsol); % getCMMdA is mexed, getCMM is not
        else
          robot.getCMM(kinsol); % getCMM is mexed, getCMMdA is not
        end
        
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
      else
        v = randn(robot.getNumVelocities(), 1);
        if include_gradients
          [~, ~, ~, ~, ~, ~] = robot.manipulatorDynamics(q, v, true);
        else
          [~, ~, ~] = robot.manipulatorDynamics(q, v, true);
        end
      end
    end
    time = toc;
    fprintf('scenario: %i, include_gradients: %i, ', scenario, include_gradients)
    fprintf('time elapsed per iteration: %0.8f\n', time / iters);
  end
end

end
