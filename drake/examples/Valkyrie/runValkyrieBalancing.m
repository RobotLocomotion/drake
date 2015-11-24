function runValkyrieBalancing(use_mex)
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

%r = Valkyrie('urdf/valkyrie_minimal_contact.urdf',options);
%fixed_point_file = fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat');

r = Valkyrie(fullfile(getDrakePath,'examples','Valkyrie','urdf','urdf','valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf'),options);
%fixed_point_file = '/home/mfallon/main-distro/software/control/matlab/data/val_description/valkyrie_fp_june2015.mat';
fixed_point_file = fullfile(getDrakePath,'examples','Valkyrie','data','valkyrie_fp_june2015_30joints_one_neck.mat');

r = r.removeCollisionGroupsExcept({'heel','toe'});
r.fixed_point_file = fixed_point_file;
r = compile(r);

nq = getNumPositions(r);

% set initial state to fixed point
load(fixed_point_file);
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

settings.r_foot_name = 'rightFoot+rightLegSixAxis_Frame+rightCOP_Frame';
settings.l_foot_name = 'leftFoot+leftLegSixAxis_Frame+leftCOP_Frame';
settings.pelvis_name = 'pelvis+leftPelvisIMU_Frame+rightPelvisIMU_Frame';

settings.r_knee_name = 'rightKneePitch';
settings.l_knee_name = 'leftKneePitch';

settings.r_akx_name = 'rightAnkleRoll';
settings.l_akx_name = 'leftAnkleRoll';

settings.r_aky_name = 'rightAnklePitch';
settings.l_aky_name = 'leftAnklePitch';

standing_plan = QPLocomotionPlanCPPWrapper(settings);

param_sets = valkyrieParams.getDefaults(r);

if use_angular_momentum
  param_sets.standing.Kp_ang = 1.0; % angular momentum proportunal feedback gain
  param_sets.standing.W_kdot = 1e-5*eye(3); % angular momentum weight
end

% Construct our control blocks
planeval = valkyrieControllers.ValkyriePlanEval(r, standing_plan);
control = valkyrieControllers.InstantaneousQPController(r, param_sets);
plancontroller = valkyrieControllers.ValkyriePlanEvalAndControlSystem(r, control, planeval);

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
