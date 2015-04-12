function benchmarkRigidBodyManipulator
%NOTEST
iters = 1000;
use_new_kinsol_options = [false true];
gradient_options = [false true];

for scenario = 1; %[1 2]
  % 1: CMM + a couple of Jacobians
  % 2: manipulatorDynamics
  
  figure(scenario);
  title(['Scenario: ' num2str(scenario)]);
  bar_data = zeros(length(use_new_kinsol_options), length(gradient_options));
  for use_new_kinsol_idx = 1 : length(use_new_kinsol_options);
    use_new_kinsol = use_new_kinsol_options(use_new_kinsol_idx);

    robot = createAtlas('rpy', struct('use_new_kinsol', use_new_kinsol));
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
    
    for include_gradients_idx = 1 : length(gradient_options)
      include_gradients = gradient_options(include_gradients_idx);

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
      time_per_iter = toc / iters;
      bar_data(include_gradients_idx, use_new_kinsol_idx) = time_per_iter;
    end
  end
  bar(bar_data);
    legend_strings = cell(length(use_new_kinsol_options), 1);
  for use_new_kinsol_idx = 1 : length(use_new_kinsol_options)
    legend_strings{use_new_kinsol_idx} = ['use new kinsol: ' num2str(use_new_kinsol_options(use_new_kinsol_idx))];
  end
  legend(legend_strings);
  
  xlabels = cell(length(gradient_options), 1);
  for gradient_options_idx = 1 : length(gradient_options)
    xlabels{gradient_options_idx} = ['gradients: ' num2str(gradient_options(gradient_options_idx))];
  end
  set(gca, 'XTickLabel', xlabels);
  ylabel('time per iteration');
end
end
