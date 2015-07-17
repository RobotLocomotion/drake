function runTrajStabilization(segment_number,passive_ankle)

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Atlasv4'));

if nargin < 1
  segment_number = -1; % do full traj
end

if nargin < 2
  passive_ankle = false;
end

options.twoD = true;
options.view = 'right';
options.floating = true;
options.ignore_self_collisions = true;
options.terrain = RigidBodyFlatTerrain();
options.enable_fastqp = false;
if passive_ankle
  s = '../urdf/atlas_simple_spring_ankle_planar_contact.urdf';
  traj_file = 'data/atlas_passiveankle_traj_lqr_zoh.mat';
  %traj_file = 'data/atlas_passiveankle_traj_lqr_090314_zoh.mat';
else  
  s = '../urdf/atlas_simple_planar_contact.urdf';
%   traj_file = 'data/atlas_lqr_fm2_cost10.mat';

% traj_file = 'data/atlas_lqr_fm2_periodic.mat';
traj_file = 'data/atlas_lqr_fm2_periodic_100.mat';
traj_file = '../data/atlas_hybrid_lqr';
  traj_file = 'data/atlas_hybrid_lqr_longer_nonperiodic';

%   traj_file = 'data/atlas_lqr_01.mat';
end
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


load(traj_file);
xtraj=xtraj{1}.append(xtraj{2}).append(xtraj{3}).append(xtraj{4}).append(xtraj{5});
utraj=utraj{1}.append(utraj{2}).append(utraj{3}).append(utraj{4}).append(utraj{5});

if true %foh on u and qd
  t_t = xtraj.pp.breaks;
  x = xtraj.eval(t_t);
  u = utraj.eval(t_t);
  qtraj = PPTrajectory(foh(t_t,x(1:r.getNumPositions,:)));
  qdtraj = PPTrajectory(zoh(t_t,[x(1+r.getNumPositions:end,2:end) zeros(r.getNumVelocities,1)]));
  xtraj = [qtraj;qdtraj];
  utraj = PPTrajectory(zoh(t_t,u));
end


repeat_n = 1;

[xtraj,utraj,Btraj,Straj_full] = repeatTraj(r,xtraj,utraj,Btraj,Straj_full,repeat_n,true);

%%% this is converting the trajectory to a zoh
% if true
%   t_t = xtraj.pp.breaks;
%   x = xtraj.eval(t_t);
%   qtraj = PPTrajectory(foh(t_t,x(1:r.getNumPositions,:)));
%   qdtraj = PPTrajectory(zoh(t_t,[x(1+r.getNumPositions:end,2:end) zeros(r.getNumVelocities,1)]));
%   xtraj = [qtraj;qdtraj];
% end

xtraj = xtraj.setOutputFrame(getStateFrame(r));
v.playback(xtraj);%,struct('slider',true));

support_times = zeros(1,length(Straj_full));
for i=1:length(Straj_full)
  support_times(i) = Straj_full{i}.tspan(1);
end
% [~,modes] = extractHybridModes(r,xtraj,support_times+0.02); % hack add time to make sure it's fully into the next mode
% 
% figure(23423);
% plot(support_times+0.03,modes,'b.-');
 
options.right_foot_name = 'r_foot';
options.left_foot_name = 'l_foot'; 
%modes = [8,6,3,3,4,4,4,2,7,7,8,8];%,8,6,3,3,4,4];
% modes = [8,6,1,3,4,4,2,1,7,8];
% modes = [8,6,3,4,4,2,7,8];
modes = [8,6,1,3,4,4,2,1,7,8];
modes = repmat(modes,1,repeat_n);
lfoot_ind = findLinkId(r,options.left_foot_name);
rfoot_ind = findLinkId(r,options.right_foot_name);  

%   mode 1: [left: heel+toe, right: heel+toe]
%   mode 2: [left: heel,     right: heel+toe]
%   mode 3: [left: toe,      right: heel+toe]
%   mode 4: [left: none,     right: heel+toe]
%   mode 5: [left: heel,     right: toe]
%   mode 6: [left: heel+toe, right: heel]
%   mode 7: [left: heel+toe, right: toe]
%   mode 8: [left: heel+toe, right: none]
%   mode 9: [left: toe,      right: heel]
%   mode 10: [left: none,    right: none]
support_states = [RigidBodySupportState(r,[lfoot_ind,rfoot_ind]); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'heel'},{'heel','toe'}}); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'toe'},{'heel','toe'}}); ...
  RigidBodySupportState(r,rfoot_ind); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'heel'},{'toe'}}); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'heel','toe'},{'heel'}}); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'heel','toe'},{'toe'}}); ...
  RigidBodySupportState(r,lfoot_ind); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],{{'toe'},{'heel'}}); ...
  RigidBodySupportState(r,[])];

supports = [];
for i=1:length(modes)
  supports = [supports; support_states(modes(i))];
end

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
options.timestep = .001;
options.dt = .001;
options.cpos_slack_limit = inf;
options.w_cpos_slack = 1.0;
options.phi_slack_limit = inf;
options.w_phi_slack = 0.0;
options.w_qdd = 0*ones(nq,1);
options.w_grf = 0;
options.Kp_accel = 0;
options.contact_threshold = 1e-4; %was 1e-4
options.offset_x = true;
qp = FullStateQPControllerDT(r,ctrl_data,options);

% feedback QP controller with Atlas
sys = feedback(r,qp);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

% t0 = .418;
% tf = 1;s

x0 = xtraj.eval(t0);
keyboard
traj = simulate(sys,[t0 tf],x0);
playback(v,traj,struct('slider',true));


if 0
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

if 0
  pptraj = PPTrajectory(foh(traj.getBreaks,traj.eval(traj.getBreaks)));
  for i=1:20
    figure(100+i);
    fnplt(xtraj(i));
    hold on;
    fnplt(pptraj(i));
    title(strrep(r.getStateFrame.coordinates(i),'_','\_'));
    hold off;
  end
end
keyboard
end

