function runTrajectoryStabilization

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

options.twoD = true;
options.view = 'right';
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
s = 'urdf/spring_flamingo_passive_ankle.urdf';
dt = 0.001;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = TimeSteppingRigidBodyManipulator(s,dt,options);
warning(w);

nx = getNumStates(r);
nq = getNumPositions(r);
nu = getNumInputs(r);

v = r.constructVisualizer;
v.display_dt = 0.01;

data_dir = fullfile(getDrakePath,'examples','SpringFlamingo','data');
traj_file = strcat(data_dir,'/traj.mat');
if exist(traj_file, 'file') ~= 2
  !wget "http://www.dropbox.com/s/i2g6fz45hl97si4/traj.mat" --no-check-certificate 
  system(['mv traj.mat ',data_dir]);
end
load(traj_file);
load('data/traj.mat');
xtraj = xtraj.setOutputFrame(getStateFrame(r));
% v.playback(xtraj,struct('slider',true));

% build support trajectory
support_times = zeros(1,length(S_full));
for i=1:length(S_full)
  support_times(i) = S_full{i}.tspan(1);
end

lfoot_ind = findLinkInd(r,'left_foot');
rfoot_ind = findLinkInd(r,'right_foot');  

support_times(2) = support_times(2);
% manually specifiy modes for now
supports = [RigidBodySupportState(r,lfoot_ind); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind]); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'toe'},{'heel','toe'}}); ...
  RigidBodySupportState(r,rfoot_ind)];
 
% heel_toe_group = {'heel','toe'};
% for i=1:length(support_times)
%   x = xtraj.eval(support_times(i));
%   q = x(1:nq);
%   kinsol = doKinematics(r,q);
%   phi = contactConstraints(r,kinsol,false,struct('terrain_only',true,...
%     'body_idx',[1,lfoot_ind,rfoot_ind],'collision_groups',{{heel_toe_group,heel_toe_group}}));
%   phi
% end

ctrl_data = FullStateQPControllerData(true,struct(...
  'B',B,...
  'S',S,...
  'R',1.25*eye(nu),... % note: TVLQR problem used R=eye(nu)
  'x0',xtraj,...
  'u0',utraj,...
  'support_times',support_times,...
  'supports',supports));

% instantiate QP controller
options.slack_limit = inf;
options.w_grf = 0.01;
options.w_slack = 0.2;
options.Kp_accel = 0.0;
options.contact_threshold = 1e-4;
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

tspan = xtraj.tspan();
traj = simulate(sys,[0 tspan(2)],xtraj.eval(tspan(1)));
playback(v,traj,struct('slider',true));

end

