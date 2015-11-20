function runAtlasBalancing(use_mex)
% put robot in a random x,y,yaw position and balance for 2 seconds

checkDependency('gurobi')

visualize = true;

if (nargin<1); use_mex = true; end
if (nargin<2); use_angular_momentum = false; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

nq = getNumPositions(r);

% set initial state to fixed point
load('data/atlas_fp.mat');
xstar(1) = 0.1*randn();
xstar(2) = 0.1*randn();
xstar(6) = pi*randn();
r = r.setInitialState(xstar);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

com = getCOM(r,kinsol);

% Construct plan
settings = QPLocomotionPlanSettings.fromStandingState(x0, r);
% settings.planned_support_command = QPControllerPlan.support_logic_maps.kinematic_or_sensed; % Only use supports when in contact
standing_plan = QPLocomotionPlanCPPWrapper(settings);

param_sets = atlasParams.getDefaults(r);

if use_angular_momentum
  param_sets.standing.Kp_ang = 1.0; % angular momentum proportunal feedback gain
  param_sets.standing.W_kdot = 1e-5*eye(3); % angular momentum weight
end

% Construct our control blocks
planeval = atlasControllers.AtlasPlanEval(r, standing_plan);
control = atlasControllers.InstantaneousQPController(r, param_sets);
plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r, control, planeval);

sys = feedback(r, plancontroller);

if visualize
  v = r.constructVisualizer;
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
if visualize
  playback(v,traj,struct('slider',true));
end

xf = traj.eval(traj.tspan(2));

err = norm(xf(1:6)-xstar(1:6))
if err > 0.02
  error('drakeBalancing unit test failed: error is too large');
end

end
