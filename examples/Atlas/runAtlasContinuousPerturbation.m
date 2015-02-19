function runAtlasWalking(perturbation, example_options)
% Example running walking QP controller from
% Scott Kuindersma, Frank Permenter, and Russ Tedrake.
% An efficiently solvable quadratic program for stabilizing dynamic
% locomotion. In Proceedings of the International Conference on 
% Robotics and Automation, Hong Kong, China, May 2014. IEEE.
%
% @option use_mex
% @option use_bullet
% @option use_angular_momentum
% @options navgoal
% 

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<2, example_options=struct(); end
if ~isfield(example_options,'use_mex'), example_options.use_mex = false; end
if ~isfield(example_options,'use_bullet') example_options.use_bullet = false; end
if ~isfield(example_options,'use_angular_momentum') example_options.use_angular_momentum = false; end
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

recovery_planner = RecoveryPlanner([], []);

zmpact = [];

combined_xtraj = [];

for iter = 1:3
  r = r.setInitialState(x0);
  v.draw(0, x0)

  plan = recovery_planner.solveBipedProblem(r, x0, zmpact);
  walking_plan_data = WalkingPlanData.from_point_mass_biped_plan(plan, r, x0);

  pm_biped = PointMassBiped(plan.omega);
  [pm_traj, pm_v] = walking_plan_data.simulatePointMassBiped(pm_biped);
  pm_v.playback(pm_traj, struct('slider', true));

  walking_plan_data.qstar = xstar(1:nq);

  ts = walking_plan_data.zmptraj.getBreaks();
  T = ts(end);

  % plot walking traj in drake viewer
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'walking-plan');

  for i=1:length(ts)
    lcmgl.glColor3f(0, 0, 1);
    lcmgl.sphere([walking_plan_data.comtraj.eval(ts(i));0], 0.01, 20, 20);
    lcmgl.glColor3f(0, 1, 0);
    lcmgl.sphere([walking_plan_data.zmptraj.eval(ts(i));0], 0.01, 20, 20);
  end
  lcmgl.switchBuffers();
  % keyboard()

  sim_opts = struct('use_mex', example_options.use_mex,...
                    'use_ik', false,...
                    'use_bullet', example_options.use_bullet,...
                    'use_angular_momentum', example_options.use_angular_momentum,...
                    'draw_button', true,...
                    'v', v);
  traj = atlasUtil.simulateWalking(r, walking_plan_data, sim_opts);

  % ctrl_data = QPControllerData(true,struct(...
  %   'acceleration_input_frame',AtlasCoordinates(r),...
  %   'D',-0.94/9.81*eye(2),... % assumed COM height
  %   'Qy',eye(2),...
  %   'S',walking_plan_data.V.S,... % always a constant
  %   's1',walking_plan_data.V.s1,...
  %   's2',walking_plan_data.V.s2,...
  %   'x0',ConstantTrajectory([walking_plan_data.zmptraj.eval(T);0;0]),...
  %   'u0',ConstantTrajectory(zeros(2,1)),...
  %   'qtraj',x0(1:nq),...
  %   'comtraj',walking_plan_data.comtraj,...
  %   'link_constraints',walking_plan_data.link_constraints, ...
  %   'support_times',walking_plan_data.support_times,...
  %   'supports',walking_plan_data.supports,...
  %   'mu',walking_plan_data.mu,...
  %   'y0',walking_plan_data.zmptraj,...
  %   'ignore_terrain',false,...
  %   'plan_shift',[0;0;0],...
  %   'constrained_dofs',[]));

  % options.dt = 0.003;
  % options.slack_limit = 100;
  % options.use_bullet = use_bullet;
  % w_qdd = zeros(nq, 1);
  % w_qdd(findJointIndices(r, 'arm')) = .0001;
  % w_qdd(findJointIndices(r, 'back')) = .0001;
  % w_qdd(findJointIndices(r, 'neck')) = .0001;
  % options.w_qdd = w_qdd;
  % options.w_grf = 1e-8;
  % options.w_slack = 5.0;
  % options.debug = true;
  % options.contact_threshold = 0.002;
  % options.solver = 0; % 0 fastqp, 1 gurobi
  % options.use_mex = use_mex;


  % options.Kp =150*ones(6,1);
  % options.Kd = 2*sqrt(options.Kp);
  % lfoot_motion = BodyMotionControlBlock(r,'l_foot',ctrl_data,options);
  % rfoot_motion = BodyMotionControlBlock(r,'r_foot',ctrl_data,options);
  % options.Kp = [100; 100; 100; 150; 150; 150];
  % options.Kd = 2*sqrt(options.Kp);
  % pelvis_motion = PelvisMotionControlBlock(r,'pelvis',ctrl_data,options);
  % % pelvis_motion.nominal_pelvis_height = 0.87;
  % motion_frames = {lfoot_motion.getOutputFrame,rfoot_motion.getOutputFrame,...
  % pelvis_motion.getOutputFrame};

  % options.body_accel_input_weights = [.001 .001 .001];
  % options.min_knee_angle = 0.7;

  % playback(v,traj,struct('slider',true));
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

  atlasUtil.plotWalkingTraj(r, traj, walking_plan_data);

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
end

combined_xtraj = combined_xtraj.setOutputFrame(r.getStateFrame());
v.playback(combined_xtraj, struct('slider', true));


% TIMEOUT 1500
