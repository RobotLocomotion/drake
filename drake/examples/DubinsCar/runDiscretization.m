function runDiscretization

  plant = DubinsPlant;
  options.dt = 1e-2;
  options.gamma = .999; % we have a discount factor to allow for convergence
  options.wrap_flag = [true;true;true];
  % we have the controller wrap around for all three state variables.

  % wrapping around in x([1, 2]) = (x, y) allows us to simulate a infinite
  % obstacle field consisting of a grid of cells, with each cell containing
  % an identical obstacle.
  % wrapping around in x(3) = θ allows the robot to complete one full
  % rotation.
  
  options.vectorized_x = false;

  cost = @dubinsSampleCost; 
  u_limit = 1;
  L = 10;
  num_linear_bins = 30; % resolution of discretization of spatial coordinates
  num_angular_bins = 65; % resolution of discretization of angular position of car
  num_action_bins = 19;

  xbins = {
      linspace(-L, L, num_linear_bins), ...
      linspace(-L, L, num_linear_bins), ...
      linspace(0, 2*pi, num_angular_bins)
          };
  ubins = linspace(-u_limit, u_limit, num_action_bins);
  mdp = MarkovDecisionProcess.discretizeSystem(plant,cost,xbins,ubins,options);

end

