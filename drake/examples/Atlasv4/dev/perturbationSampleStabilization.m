function runTrajStabilization3D(segment_number)

if nargin <1
  segment_number=-1;
end

if ~checkDependency('gurobi')
  warning('Must have gurobi installed to run this example');
  return;
end

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Atlasv4'));

options.floating = true;
options.ignore_self_collisions = true;
options.enable_fastqp = false;

s = '../urdf/atlas_simple_contact_noback.urdf';
% traj_file = 'data/atlas_3d_lqr2.mat'; 
traj_file = 'data/longer_step_smooth_lqr.mat'; 
options.terrain = RigidBodyFlatTerrain();
modes = [8,3,4,4,7,8];

w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = Atlas(s,options);
warning(w);

nx = getNumStates(r);
nq = getNumPositions(r);
nu = getNumInputs(r);

v = r.constructVisualizer;
v.display_dt = 0.005;

load(traj_file);

repeat_n = 1;
[xtraj,utraj,Btraj,Straj_full] = repeatTraj(r,xtraj,utraj,Btraj,Straj_full,repeat_n,true,true);


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
%modes = [8,6,1,3,4,4,2,1,7,8];


if repeat_n >1
  modes = repmat(modes,1,repeat_n-1);
end
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
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],struct('contact_groups',{{{'heel'},{'heel','toe'}}})); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],struct('contact_groups',{{{'toe'},{'heel','toe'}}})); ...
  RigidBodySupportState(r,rfoot_ind); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],struct('contact_groups',{{{'heel'},{'toe'}}})); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],struct('contact_groups',{{{'heel','toe'},{'heel'}}})); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],struct('contact_groups',{{{'heel','toe'},{'toe'}}})); ...
  RigidBodySupportState(r,lfoot_ind); ...
  RigidBodySupportState(r,[lfoot_ind,rfoot_ind],struct('contact_groups',{{{'toe'},{'heel'}}})); ...
  RigidBodySupportState(r,[])];

supports = [];
for i=1:length(modes)
  supports = [supports; support_states(modes(i))];
end

if segment_number<1
  B=Btraj;
  S=Straj_full;
  if iscell(xtraj)
    t0 = xtraj{1}.tspan(1);
    tf = xtraj{length(xtraj)}.tspan(2);
  else
    t0 = xtraj.tspan(1);
    tf = xtraj.tspan(2);
    v.playback(xtraj);%,struct('slider',true));
  end
else
  B=Btraj{segment_number};
  S=Straj_full{segment_number};
  t0 = round(1000*Btraj{segment_number}.tspan(1))/1000;
  tf = Btraj{segment_number}.tspan(2);
  if iscell(xtraj)
    xtraj = xtraj{segment_number};
    utraj = utraj{segment_number};
  end
  xtraj = xtraj.setOutputFrame(getStateFrame(r));
  v.playback(xtraj);%,struct('slider',true));
end

allowable_supports = RigidBodySupportState(r,[lfoot_ind,rfoot_ind]);

ctrl_data = FullStateQPControllerData(true,struct(...
  'B',{B},...
  'S',{S},...
  'R',R,... 
  'x0',{xtraj},...
  'u0',{utraj},...
  'support_times',support_times,...
  'supports',supports,...
  'allowable_supports',allowable_supports));

% instantiate QP controller
options.timestep = .001;
options.dt = .001;
options.cpos_slack_limit = inf;
options.w_cpos_slack = 1;
options.phi_slack_limit = inf;
options.w_phi_slack = 0.0;
options.w_qdd = 0*ones(nq,1);
options.w_grf = 0;
options.Kp_accel = 0;
options.contact_threshold = 5e-4;
options.offset_x = true;
qp = FullStateQPController(r,ctrl_data,options);

% feedback QP controller with Atlas
sys = feedback(r,qp);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

% t0 = .418;
% tf = 1;s
if iscell(xtraj)
  x0 = xtraj{1}.eval(t0);
else
  x0 = xtraj.eval(t0);
end
x0_ = x0;

x0_nom = x0;
q0 = x0_nom(1:r.getNumPositions);
qd0 = x0_nom(r.getNumPositions+1:end)
[H,C,B] = r.manipulatorDynamics(q0,qd0);
nom_KE = qd0'*H*qd0
[phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = contactConstraints(r,q0);
J = [n(1:4,:);D{1}(1:4,:);D{2}(1:4,:)];
keyboard
%% Perturb x0
global cost_t
for i=1:100,
f = randn(3,1);
f = 10*f/norm(f);
f = [f;zeros(r.getNumPositions-3,1)];
Lambda = -pinv(J*inv(H)*J')*J*inv(H)*f;
delta_qd = inv(H)*(J'*Lambda + f);

x0(r.getNumPositions+1:end) = x0_nom(r.getNumPositions+1:end) + delta_qd;
traj = simulate(sys,[t0 tf],x0);

initial_cost(i) = (x0-x0_)'*Straj_full{1}.eval(0)*(x0-x0_);
final_cost(i) = cost_t;
perturb{i}.traj = traj;
perturb{i}.f = f;
end
% KE=x0(r.getNumPositions+1:end)'*H*x0(r.getNumPositions+1:end)

%%
traj = simulate(sys,[t0 tf],x0);
playback(v,traj,struct('slider',true));


% save('data/atlas_3d_traj_exec.mat','xtraj','traj');



if 0
  traj_ts = traj.getBreaks();
  traj_pts = traj.eval(traj_ts);
  
  if iscell(xtraj)
    xtraj_cell = xtraj;
    xtraj = xtraj_cell{1};
    for i=2:length(xtraj_cell);
      xtraj=xtraj.append(xtraj_cell{i});
    end
  end
  
  xtraj_pts = xtraj.eval(traj_ts);
  
  figure(111);
  clf
  max_y = -inf;
  min_y = inf;
  for i=1:nq
    subplot(2,ceil(nq/2),i);
%     figure(111+i);
    hold on;
    title(r.getStateFrame.coordinates{i},'interpreter','none');
%     plot(traj_ts,xtraj_pts(i,:),'g.-');
%     plot(traj_ts,traj_pts(i,:),'r.-');
    y = (traj_pts(i,:)-xtraj_pts(i,:));
    plot(traj_ts,y,'r.-');
    max_y = max(max_y,max(y));
    min_y = min(min_y,min(y));
    hold off;
  end
  for i=1:nq
    subplot(2,ceil(nq/2),i);
    ylim([min_y max_y])
  end
  
  figure(112);
  clf
  max_y = -inf;
  min_y = inf;
  for i=1:nq
    subplot(2,ceil(nq/2),i);
%     figure(555+i);
    hold on;
    title(r.getStateFrame.coordinates{nq+i},'interpreter','none');
%     plot(traj_ts,xtraj_pts(nq+i,:),'g.-');
%     plot(traj_ts,traj_pts(nq+i,:),'r.-');
    y = (traj_pts(nq+i,:)-xtraj_pts(nq+i,:));
    plot(traj_ts,y,'r.-');
    max_y = max(max_y,max(y));
    min_y = min(min_y,min(y));
    hold off;
  end
  for i=1:nq
    subplot(2,ceil(nq/2),i);
    ylim([min_y max_y])
  end
  
  global utraj_pts utraj_ts
  
  if iscell(utraj)
    utraj_cell = utraj;
    utraj = utraj_cell{1};
    for i=2:length(utraj_cell);
      utraj=utraj.append(utraj_cell{i});
    end
  end
  
  u0_pts = utraj.eval(traj_ts);
  
  figure(116);
  clf
  for i=1:nu
    subplot(2,ceil(nu/2),i);
    hold on;
    title(r.getInputFrame.coordinates{i});
    plot(traj_ts,u0_pts(i,:),'g.-');
    plot(utraj_ts,utraj_pts(i,:),'r.-');
    hold off;
  end
end
keyboard
end

