function runTrajStab(segment_number)

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end
addpath(fullfile(getDrakePath,'examples','SpringFlamingo'));

if nargin < 1
  segment_number = -1; % do full traj
end

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.twoD = true;
options.view = 'right';
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
s = 'OneLegHopper.urdf';
dt = 0.001;
r = TimeSteppingRigidBodyManipulator(s,dt,options);
r = r.setStateFrame(OneLegHopperState(r));
r = r.setOutputFrame(OneLegHopperState(r));

nx = getNumStates(r);
nq = getNumPositions(r);
nu = getNumInputs(r);

v = r.constructVisualizer;
v.display_dt = 0.01;

load('data/hopper_traj_lqr_working.mat');

[xtraj,utraj,Btraj,Straj,Straj_full] = repeatTraj(xtraj,utraj,Btraj,Straj,Straj_full,1);

xtraj = xtraj.setOutputFrame(getStateFrame(r));
v.playback(xtraj);

support_times = zeros(1,length(Straj_full));
for i=1:length(Straj_full)
  support_times(i) = Straj_full{i}.tspan(1);
end

options.right_foot_name = 'foot';
options.left_foot_name = 'thigh'; % junk for now

foot_ind = findLinkId(r,'foot');

[ts,modes] = extractHybridModes(r,xtraj);

support_times = ts([1 find(diff(modes))]);
modes = modes([1 1+find(diff(modes))]);

if length(support_times) ~= length(modes)
  disp('these should be equal');
  keyboard;
end

supports = [];
for i=1:length(modes)
  switch modes(i)
    case 1
      supp = RigidBodySupportState(r,foot_ind);
    case 2
      supp = RigidBodySupportState(r,foot_ind,{{'heel'}}); 
    case 3
      supp = RigidBodySupportState(r,foot_ind,{{'toe'}}); 
    case 4
      supp = RigidBodySupportState(r,[]);
  end
  supports = [supports; supp];
end

% % manually specifiy modes for now
% supports = [RigidBodySupportState(r,foot_ind); ...
%   RigidBodySupportState(r,foot_ind,{{'toe'}}); ...
%   RigidBodySupportState(r,[]); ...
%   RigidBodySupportState(r,foot_ind)];

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
options.slack_limit = 10;
options.w_qdd = 0*ones(nq,1);
options.w_grf = 0;
options.w_slack = 1.0;
options.Kp_accel = 0.0;
options.contact_threshold = 1e-3;
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
traj = simulate(sys,[t0 tf],xtraj.eval(t0));
playback(v,traj,struct('slider',true));

if 0
  % plot position tracking
  pptraj = PPTrajectory(foh(traj.getBreaks,traj.eval(traj.getBreaks)));
  for i=1:10
    figure(100+i);
    hold on;
    fnplt(xtraj(i));
    fnplt(pptraj(i));
    hold off;
  end
end

if 1
  % plot mode sequence
  pptraj = PPTrajectory(foh(traj.getBreaks,traj.eval(traj.getBreaks)));
  
  [ts,modes] = extractHybridModes(r,xtraj);
  [ts_,modes_] = extractHybridModes(r,pptraj);

  figure(999);
  plot(ts,modes,'b');
  hold on;
  plot(ts_,modes_,'r');
  hold off;
end

end

