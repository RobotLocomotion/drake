function traj=passiveAnkleExampleStabilization(xtraj,utraj,Btraj,Straj_full,R)

if ~checkDependency('gurobi')
  error('Must have gurobi installed to run this example');
end

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Atlasv4'));

options.twoD = true;
options.view = 'right';
options.floating = true;
options.ignore_self_collisions = true;
options.enable_fastqp = false;

s = '../urdf/atlas_simple_spring_ankle_planar_contact.urdf';
%traj_file = 'data/atlas_passiveankle_traj_lqr_090314_zoh.mat';
options.terrain = RigidBodyFlatTerrain();

% really should extract the mode sequence from the optimization scripts
% corresponds to the mode list below
modes = [8,3,4,4,7,8]; 
contact_seq_full = {[1;2], [2;3;4], [3;4], [3;4], [1;2;4], [1;2]};

w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = Atlas(s,options);

r = r.setOutputFrame(AtlasXZState(r));
r = r.setStateFrame(AtlasXZState(r));
warning(w);

nx = getNumStates(r);
nq = getNumPositions(r);
nu = getNumInputs(r);

v = r.constructVisualizer(struct('viewer','BotVisualizer'));
v.display_dt = 0.01;

  repeat_n = 4;
  [xtraj,utraj,Btraj,Straj_full] = repeatTraj(r,xtraj,utraj,Btraj,Straj_full,repeat_n,true,false);

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


modes = repmat(modes,1,repeat_n-1);
contact_seq_full = repmat(contact_seq_full,1,repeat_n-1);
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
%   mode 11: [left: heel+toe, right: none,     hand: true]
%   mode 12: [left: none,     right: heel+toe, hand: true]
%   mode 13: [left: heel+toe, right: heel+toe, hand: true]
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

  allowable_supports = RigidBodySupportState(r,[lfoot_ind,rfoot_ind]);

supports = [];
for i=1:length(modes)
  supports = [supports; support_states(modes(i))];
end

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

ctrl_data = FullStateQPControllerData(true,struct(...
  'B',{B},...
  'S',{S},...
  'R',R,... 
  'x0',{xtraj},...
  'u0',{utraj},...
  'support_times',support_times,...
  'supports',supports,...
  'allowable_supports',allowable_supports));

ctrl_data.contact_seq = contact_seq_full;

% instantiate QP controller
options.timestep = .001;
options.dt = .001;
options.cpos_slack_limit = inf;
options.w_cpos_slack = 0.1;
options.phi_slack_limit = inf;
options.w_phi_slack = 0.0;
options.w_qdd = 0*ones(nq,1);
options.w_grf = 0;
options.Kp_accel = 0;
options.contact_threshold = 5e-4; %was 1e-4
options.offset_x = true;
qp = FullStateQPController(r,ctrl_data,options);
% qp = FullStateQPControllerDT(r,ctrl_data,options);

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
traj = simulate(sys,[t0 tf],x0);
playback(v,traj,struct('slider',true));

if 1
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
  for i=1:nq
    subplot(2,ceil(nq/2),i);
    hold on;
    title(r.getStateFrame.getCoordinateNames{i});
    plot(traj_ts,xtraj_pts(i,:),'g.-');
    plot(traj_ts,traj_pts(i,:),'r.-');
    hold off;
  end
  figure(112);
  for i=1:nq
    subplot(2,ceil(nq/2),i);
    hold on;
    title(r.getStateFrame.getCoordinateNames{nq+i});
    plot(traj_ts,xtraj_pts(nq+i,:),'g.-');
    plot(traj_ts,traj_pts(nq+i,:),'r.-');
    hold off;
  end
end
save('data/atlas_passive_walking_exec.mat','traj');

end

