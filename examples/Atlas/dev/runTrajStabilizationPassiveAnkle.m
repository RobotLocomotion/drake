function runTrajStabilizationPassiveAnkle(segment_number)

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

path_handle = addpathTemporary({fullfile(getDrakePath,'examples','Atlas','controllers'),...
                                fullfile(getDrakePath,'examples','Atlas'),...
                                fullfile(getDrakePath,'examples','Atlas','frames')});
if nargin < 1
  segment_number = -1; % do full traj
end

options.twoD = true;
options.view = 'right';
options.floating = true;
options.ignore_self_collisions = true;
options.terrain = RigidBodyFlatTerrain();
s = '../urdf/atlas_simple_spring_ankle_planar_contact.urdf';
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = Atlas(s,options);
r = r.setOutputFrame(AtlasXZState(r));
r = r.setStateFrame(AtlasXZState(r));
warning(w);

nx = getNumStates(r);
nq = getNumPositions(r);
nu = getNumInputs(r);

v = r.constructVisualizer;
v.display_dt = 0.01;

traj_file = 'data/atlas_passiveankle_traj_lqr_090314_zoh.mat';
load(traj_file);

[xtraj,utraj,Btraj,Straj_full] = repeatTraj(r,xtraj,utraj,Btraj,Straj_full,2,true);

%%% this is converting the trajectory to a zoh
% if true
%   t_t = xtraj.pp.breaks;
%   x = xtraj.eval(t_t);
%   qtraj = PPTrajectory(foh(t_t,x(1:r.getNumPositions,:)));
%   qdtraj = PPTrajectory(zoh(t_t,[x(1+r.getNumPositions:end,2:end) zeros(r.getNumVelocities,1)]));
%   xtraj = [qtraj;qdtraj];
% end

xtraj = xtraj.setOutputFrame(getStateFrame(r));
% v.playback(xtraj,struct('slider',true));

support_times = zeros(1,length(Straj_full));
for i=1:length(Straj_full)
  support_times(i) = Straj_full{i}.tspan(1);
end

options.right_foot_name = 'r_foot';
options.left_foot_name = 'l_foot'; 

lfoot_ind = findLinkId(r,options.left_foot_name);
rfoot_ind = findLinkId(r,options.right_foot_name);  

% manually specifiy modes for now
% supports = [RigidBodySupportState(r,lfoot_ind); ...
%   RigidBodySupportState(r,[lfoot_ind,rfoot_ind]); ...
%   RigidBodySupportState(r,rfoot_ind); ...
%   RigidBodySupportState(r,rfoot_ind)];
% supports = [RigidBodySupportState(r,lfoot_ind); ...
%   RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'heel','toe'},{'heel'}}); ...
%   RigidBodySupportState(r,[lfoot_ind,rfoot_ind]); ...
%   RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'toe'},{'toe','heel'}});...
%   RigidBodySupportState(r,rfoot_ind)];
supports_left = [RigidBodySupportState(r,lfoot_ind); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'heel','toe'},{'heel'}}); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'toe'},{'heel'}}); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'toe'},{'heel','toe'}}); ...
  RigidBodySupportState(r,rfoot_ind)];

supports_right = [RigidBodySupportState(r,rfoot_ind); ...
  RigidBodySupportState(r,[rfoot_ind,lfoot_ind],{{'heel','toe'},{'heel'}}); ...
  RigidBodySupportState(r,[rfoot_ind,lfoot_ind],{{'toe'},{'heel'}}); ...
  RigidBodySupportState(r,[rfoot_ind,lfoot_ind],{{'toe'},{'heel','toe'}}); ...
  RigidBodySupportState(r,lfoot_ind)];

supports = [supports_left; supports_right; supports_left; supports_right];


if segment_number<1
  B=Btraj;
  S=Straj_full;
  t0 = xtraj.tspan(1);
  tf = xtraj.tspan(2);
else
  B=Btraj{segment_number};
  S=Straj_full{segment_number};
  t0 = Btraj{segment_number}.tspan(1);
  tf = Btraj{segment_number}.tspan(2); 
end

ctrl_data = FullStateQPControllerData(true,struct(...
  'B',{B},...
  'S',{S},...
  'R',R,... 
  'x0',xtraj,...
  'u0',utraj,...
  'support_times',support_times,...
  'supports',supports));

% instantiate QP controller
options.slack_limit = inf;
options.w_qdd = 0.0*ones(nq,1);
options.w_grf = 0.0;
options.w_slack = 0.0;
options.Kp_accel = 0.0;
options.contact_threshold = 0.001;
qp = FullStateQPController(r,ctrl_data,options);

% feedback QP controller with spring flamingo
ins(1).system = 1;
ins(1).input = 2;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);

% feedback foot contact detector 
options.use_contact_logic_OR = false;
fc = FootContactBlock(r,ctrl_data,options);
sys = mimoFeedback(fc,sys,[],[],[],outs);
  
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

x0 = xtraj.eval(t0);

traj = simulate(sys,[t0 tf],xtraj.eval(t0));
playback(v,traj,struct('slider',true));

end

