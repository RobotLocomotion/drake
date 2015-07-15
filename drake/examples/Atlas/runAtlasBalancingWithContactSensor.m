function runAtlasBalancingWithContactSensor(example_options)
if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'ZMP'));
import atlasControllers.*;

% put robot in a random x,y,yaw position and balance for 2 seconds
if nargin<1, example_options=struct(); end
example_options = applyDefaults(example_options, struct('use_mex', true, ...
                                                        'use_bullet', false,...
                                                        'visualize', true,...
                                                        'terrain', RigidBodyFlatTerrain));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.ignore_self_collisions = true;
options.ignore_friction = true;
options.dt = 0.001;
options.terrain = example_options.terrain;
options.use_bullet = example_options.use_bullet;
options.use_mex = example_options.use_mex;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

r_pure = r;
l_foot_body = findLinkId(r,'l_foot');
l_foot_frame = RigidBodyFrame(l_foot_body,zeros(3,1),zeros(3,1),'l_foot_forcetorque');
r = r.addFrame(l_foot_frame);
l_foot_force_sensor = ContactForceTorqueSensor(r, l_foot_frame);
r = addSensor(r, l_foot_force_sensor);
r_foot_body = findLinkId(r,'r_foot');
r_foot_frame = RigidBodyFrame(r_foot_body,zeros(3,1),zeros(3,1),'r_foot_forcetorque');
r = r.addFrame(r_foot_frame);
r_foot_force_sensor = ContactForceTorqueSensor(r, r_foot_frame);
r = addSensor(r, r_foot_force_sensor);

r = compile(r);


nq = getNumPositions(r);

% set initial state to fixed point
load('data/atlas_fp.mat');
xstar(1) = 0.1*randn();
xstar(2) = 0.1*randn();
xstar(6) = pi*randn();
initstate = zeros(r.getNumStates, 1);
initstate(1:length(xstar)) = xstar;
r = r.setInitialState(initstate);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

standing_plan = QPLocomotionPlanSettings.fromStandingState(x0, r);
% standing_plan.planned_support_command = QPControllerPlan.support_logic_maps.kinematic_or_sensed;

control = atlasControllers.InstantaneousQPController(r_pure, [], struct());
planeval = atlasControllers.AtlasPlanEval(r_pure, QPLocomotionPlanCPPWrapper(standing_plan));

plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r_pure, control, planeval);

% Pass through outputs from robot
outs(1).system = 1;
outs(1).output = 1;
outs(2).system = 1;
outs(2).output = 2;
outs(3).system = 1;
outs(3).output = 3;
sys = mimoFeedback(r, plancontroller, [], [], [], outs);

if example_options.visualize
  v = r.constructVisualizer;
  v.display_dt = 0.01;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  output_select(2).system=1;
  output_select(2).output=2;
  output_select(3).system=1;
  output_select(3).output=3;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end
x0 = initstate;
% x0(3) = 1.0; % drop it a bit

traj = simulate(sys,[0 2.0],x0);
if example_options.visualize
  playback(v,traj,struct('slider',true));
end

xf = traj.eval(traj.tspan(2));

err = norm(xf(1:6)-xstar(1:6))
if err > 0.02
  error('drakeBalancing unit test failed: error is too large');
end

end
