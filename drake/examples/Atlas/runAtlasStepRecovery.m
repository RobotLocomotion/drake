function runAtlasStepRecovery(perturbation, example_options)
% NOTEST
% This is an initial test of our newly developed recovery planner. It starts
% the robot with an initial perturbation and attempts to come to rest standing
% up. 
% @param perturbation the disturbance, expressed as initial CoM velocity in [x; y]
% @option use_bullet
% @option use_angular_momentum
% @options navgoal
% 

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<2, example_options=struct(); end
if ~isfield(example_options,'use_bullet'), example_options.use_bullet = false; end
if ~isfield(example_options,'use_angular_momentum'), example_options.use_angular_momentum = false; end
if ~isfield(example_options,'navgoal')
  example_options.navgoal = [1.5;0;0;0;0;0];
end
if ~isfield(example_options,'terrain'), example_options.terrain = RigidBodyFlatTerrain; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
options.ignore_friction = true;
options.dt = 0.001;
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
% xstar(r.getNumPositions() + (1:2)) = [0.3; 0.3];
xstar = Point(r.getStateFrame(), xstar);
% xstar.r_leg_hpy = -0.9;
% xstar.r_leg_kny = 1.7;
% xstar = double(xstar);
x0 = xstar;
nq = r.getNumPositions();
x0(nq + (1:2)) = x0(nq + (1:2)) + perturbation;

v = r.constructVisualizer;
v.display_dt = 0.001;

recovery_planner = RecoveryPlanner([], [], false);

zmpact = [];

combined_xtraj = [];

for iter = 1:3
  r = r.setInitialState(x0);
  v.draw(0, x0)

  % profile on
  [walking_plan_data, recovery_plan] = planning_pipeline(recovery_planner, r, x0, zmpact, xstar, nq);
  % profile viewer

  walking_plan_data.robot = r;

  pm_biped = PointMassBiped(recovery_plan.omega);
  [pm_traj, pm_v] = walking_plan_data.simulatePointMassBiped(pm_biped);
  pm_v.playback(pm_traj, struct('slider', true));


  ts = walking_plan_data.zmptraj.getBreaks();

  % plot walking traj in drake viewer
  lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'walking-plan');

  for i=1:length(ts)
    lcmgl.glColor3f(0, 0, 1);
    lcmgl.sphere([walking_plan_data.comtraj.eval(ts(i));0], 0.01, 20, 20);
    lcmgl.glColor3f(0, 1, 0);
    lcmgl.sphere([walking_plan_data.zmptraj.eval(ts(i));0], 0.01, 20, 20);
  end
  lcmgl.switchBuffers();
  % keyboard()

  planeval = bipedControllers.BipedPlanEval(r, walking_plan_data);
  control = bipedControllers.InstantaneousQPController(r.getManipulator().urdf{1}, r.control_config_file, fullfile(getDrakePath(), 'examples', 'Atlas', 'config', 'urdf_modifications_no_hands.yaml'));
  plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(r, control, planeval);
  sys = feedback(r, plancontroller);

  output_select(1).system=1;
  output_select(1).output=1;
  sys = mimoCascade(sys,v,[],[],output_select);

  T = walking_plan_data.duration;

  traj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));

  playback(v,traj,struct('slider',true));
  
  tsample = 0:0.01:(T-0.01);
  rsole = zeros(6, length(tsample));
  lsole = zeros(6, length(tsample));
  for j = 1:length(tsample)
    t = tsample(j);
    x = traj.eval(t);
    kinsol = r.doKinematics(x(1:r.getNumPositions()));
    rsole(:,j) = r.forwardKin(kinsol, r.foot_frame_id.right, [0;0;0], 1);
    lsole(:,j) = r.forwardKin(kinsol, r.foot_frame_id.left, [0;0;0], 1);
  end
  figure(321)
  subplot 212
  hold on
  plot(tsample, rsole(3,:), 'b.-')
  plot(tsample, lsole(3,:), 'g.-')
  xlim([0, T]);

  x0 = traj.eval(tsample(end));

  r.plotWalkingTraj(traj, walking_plan_data);

  % keyboard();

  breaks = traj.getBreaks();
  traj = PPTrajectory(foh(breaks, traj.eval(breaks)));
  if isempty(combined_xtraj)
    combined_xtraj = traj;
  else
    b = combined_xtraj.getBreaks();
    traj = traj.shiftTime(b(end)-breaks(1));
    combined_xtraj = combined_xtraj.append(traj);
  end
  break;
end

combined_xtraj = combined_xtraj.setOutputFrame(r.getStateFrame());
v.playback(combined_xtraj, struct('slider', true));


% TIMEOUT 1500

end

function [walking_plan_data, recovery_plan] = planning_pipeline(recovery_planner, r, x0, zmpact, xstar, nq)
  % Put into its own function to make profiling easier
  t0 = tic();
  recovery_plan = recovery_planner.solveBipedProblem(r, x0, zmpact, 0);
  toc(t0);
  walking_plan_data = QPLocomotionPlan.from_point_mass_biped_plan(recovery_plan, r, x0);
  walking_plan_data.qtraj = xstar(1:nq);
  % walking_plan_data = DRCWalkingPlanData.from_walking_plan_t(walking_plan_data.to_walking_plan_t()); 
end
