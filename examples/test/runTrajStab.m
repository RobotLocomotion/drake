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
options.enable_fastqp = false;
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

load('data/hopper_hybrid_lqr_sk.mat');
%load('data/hopper_traj_lqr_refined.mat');

N_hops = 10;

[xtraj,utraj,Btraj,Straj,Straj_full] = repeatTraj(xtraj,utraj,Btraj,Straj,Straj_full,N_hops);
%mode_data = repmat(mode_data,1,N_hops+1);

support_times = zeros(1,length(Straj_full));
for i=1:length(Straj_full)
  support_times(i) = Straj_full{i}.tspan(1);
end

options.right_foot_name = 'foot';
options.left_foot_name = 'thigh'; % junk for now

foot_ind = findLinkId(r,'foot');

if segment_number<1
  B=Btraj;
  S=Straj_full;
  if iscell(xtraj)
    t0 = xtraj{1}.tspan(1);
    tf = xtraj{length(xtraj)}.tspan(2);
  else
    t0 = xtraj.tspan(1);
    tf = xtraj.tspan(2);
    v.playback(xtraj);
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
  v.playback(xtraj);
end



% if iscell(xtraj)
%   modes = [];
%   for i=1:length(xtraj)
%     [~,mode_i] = extractHybridModes(r,xtraj{i},xtraj.tspan(1)+0.02); % hack add time to make sure it's fully into the next mode
%   end
%   modes = [modes mode_i];
% else
%  [~,modes] = extractHybridModes(r,xtraj,support_times+0.03); % hack add time to make sure it's fully into the next mode
% end

modes = repmat([1 3 4 1],1,N_hops);

% support_times = ts([1 find(diff(modes_))]);
% modes = modes_([1 1+find(diff(modes_))]);

if length(support_times) ~= length(Straj)
  disp('these should be equal');
  keyboard;
end

allowable_supports = RigidBodySupportState(r,foot_ind);

supports = [];
for i=1:length(modes)
  switch modes(i)
    case 1
      supp = RigidBodySupportState(r,foot_ind);
    case 2
      options.contact_groups = {{'heel'}};
      supp = RigidBodySupportState(r,foot_ind,options); 
    case 3
      options.contact_groups = {{'toe'}};
      supp = RigidBodySupportState(r,foot_ind,options); 
    case 4
      options.contact_groups = [];
      supp = RigidBodySupportState(r,[]);
  end
  supports = [supports; supp];
end

% % manually specifiy modes for now
% supports = [RigidBodySupportState(r,foot_ind); ...
%   RigidBodySupportState(r,foot_ind,{{'toe'}}); ...
%   RigidBodySupportState(r,[]); ...
%   RigidBodySupportState(r,foot_ind)];

ctrl_data = FullStateQPControllerData(true,struct(...
  'B',{B},...
  'S',{S},...
  'R',R,... 
  'x0',{xtraj},...
  'u0',{utraj},...
  'support_times',support_times,...
  'supports',supports,...
  'allowable_supports',allowable_supports));

%ctrl_data.mode_data = mode_data;

% instantiate QP controller
options.cpos_slack_limit = inf;
options.w_cpos_slack = 0.1;
options.phi_slack_limit = inf;
options.w_phi_slack = 0.0;
options.w_qdd = 1e-10*ones(nq,1);
options.w_grf = 1e-10;
options.Kp_accel = 0;
options.Kp_phi = 0;
options.contact_threshold = 1e-3;
options.timestep = 0.001;
options.offset_x = false;
qp = FullStateQPController(r,ctrl_data,options);

% feedback QP controller with spring flamingo
sys = feedback(r,qp);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

if iscell(xtraj)
  x0=xtraj{1}.eval(t0);  
else
  x0=xtraj.eval(t0);  
end
traj = simulate(sys,[t0 tf],x0);
playback(v,traj,struct('slider',true));


if 1
  traj_ts = traj.getBreaks();
  traj_pts = traj.eval(traj_ts);
  
  figure(100+i);
  if ~iscell(xtraj)
    xtraj_pts = xtraj.eval(traj_ts);
    for i=1:10
      subplot(2,5,i);
      hold on;
      title(r.getStateFrame.coordinates{i});
      plot(traj_ts,xtraj_pts(i,:),'b.-');
      plot(traj_ts,traj_pts(i,:),'r.-');
      hold off;
    end
  else
    for i=1:10
      figure(100+i);
      hold on;
      title(r.getStateFrame.coordinates{i});
      for j=1:length(xtraj)
        fnplt(xtraj{j},i);
      end
      plot(traj_ts,traj_pts(i,:),'r.-');
      hold off;
    end    
  end

  
end

save('data/hopper_traj.mat','traj');

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

end

