function runTrajectoryStabilization(segment_number)

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

if nargin < 1
  segment_number = -1; % do full traj
end

options.twoD = true;
options.view = 'right';
options.floating = true;
options.ignore_self_collisions = true;
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
traj_file = strcat(data_dir,'/traj-08-06-14.mat');
% if exist(traj_file, 'file') ~= 2
%   !wget "http://www.dropbox.com/s/i2g6fz45hl97si4/traj.mat" --no-check-certificate 
%   system(['mv traj.mat ',data_dir]);
% end
load(traj_file);
xtraj = repeatTraj(r,xtraj,10,true);
xtraj = xtraj.setOutputFrame(getStateFrame(r));
 v.playback(xtraj,struct('slider',true));

support_times = zeros(1,length(Straj_full));
for i=1:length(Straj_full)
  support_times(i) = Straj_full{i}.tspan(1);
end

lfoot_ind = findLinkInd(r,'left_foot');
rfoot_ind = findLinkInd(r,'right_foot');  

support_times(2) = support_times(2);
% manually specifiy modes for now
supports = [RigidBodySupportState(r,lfoot_ind); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind]); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'toe'},{'heel','toe'}}); ...
  RigidBodySupportState(r,rfoot_ind)];

if segment_number<1
  B=Btraj;
  S=Straj_full;
  t0 = xtraj.tspan(1);
  tf = xtraj.tspan(2);
else
  B=Btraj{segment_number};
  S=Straj_full{segment_number};
  t0 = Btraj{segment_number}.tspan(1)+0.1;
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
options.w_qdd = 0.001*ones(nq,1);
options.w_grf = 0.0;
options.w_slack = 0.0;
options.Kp_accel = 0.0;
options.contact_threshold = 1e-2;
qp = FullStateQPController(r,ctrl_data,options);

% feedback QP controller with spring flamingo
ins(1).system = 1;
ins(1).input = 2;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);

% feedback foot contact detector 
options.use_contact_logic_OR = true;
fc = FootContactBlock(r,ctrl_data,options);
sys = mimoFeedback(fc,sys,[],[],[],outs);
  
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

tspan = xtraj.tspan();
traj = simulate(sys,[t0 tf],xtraj.eval(t0));
playback(v,traj,struct('slider',true));

end

