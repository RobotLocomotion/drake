function runBalancingDemo(obj, options)
% put robot in a random x,y,yaw position and balance for 2 seconds

if nargin < 2
  options = struct();
end

options = applyDefaults(options, struct('use_angular_momentum', false,...
                                        'visualize', true));

checkDependency('gurobi')

% set initial state to fixed point
load(obj.fixed_point_file, 'xstar');
xstar(1) = 0.1*randn();
xstar(2) = 0.1*randn();
xstar(6) = pi*randn();
obj = obj.setInitialState(xstar);

% Construct plan
settings = QPLocomotionPlanSettings.fromStandingState(xstar, obj);
% settings.planned_support_command = QPControllerPlan.support_logic_maps.kinematic_or_sensed; % Only use supports when in contact
standing_plan = QPLocomotionPlanCPPWrapper(settings);

if options.use_angular_momentum
  error('use_angular_momentum setting no longer supported from Matlab');
  % param_sets.standing.Kp_ang = 1.0; % angular momentum proportunal feedback gain
  % param_sets.standing.W_kdot = 1e-5*eye(3); % angular momentum weight
end

% Construct our control blocks
planeval = bipedControllers.BipedPlanEval(obj, standing_plan);
control = bipedControllers.InstantaneousQPController(obj.getManipulator().urdf{1}, obj.control_config_file);
plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(obj, control, planeval);

sys = feedback(obj, plancontroller);

if options.visualize
  v = obj.constructVisualizer;
  v.display_dt = 0.01;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end
x0 = xstar;
% x0(3) = 1.0; % drop it a bit

traj = simulate(sys,[0 2],x0);
if options.visualize
  playback(v,traj,struct('slider',true));
end

xf = traj.eval(traj.tspan(2));

err = norm(xf(1:6)-xstar(1:6))
if err > 0.02
  error('drakeBalancing unit test failed: error is too large');
end

end
