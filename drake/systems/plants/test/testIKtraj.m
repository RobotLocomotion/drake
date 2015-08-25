function testIKtraj()
options.floating = true;
options.dt = 0.001;
r = RigidBodyManipulator();
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = r.addRobotFromURDF('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf',[],[],options);
warning(w);
nom_data = load('../../../examples/Atlas/data/atlas_fp.mat');
v = r.constructVisualizer();

r_foot = r.findLinkId('r_foot');
l_foot = r.findLinkId('l_foot');
r_hand = r.findLinkId('r_hand');
l_hand = r.findLinkId('l_hand');
head = r.findLinkId('head');
pelvis = r.findLinkId('pelvis');


l_hand_pts = [0;0.2;0];
r_hand_pts = [0;0.2;0];

r_foot_contact_pts = r.getBody(r_foot).getTerrainContactPoints();
l_foot_contact_pts = r.getBody(l_foot).getTerrainContactPoints();
r_hand_pts = mean(r_hand_pts,2);
l_hand_pts = mean(l_hand_pts,2);

r_foot_pts = [0;0;0];
l_foot_pts = [0;0;0];


nq = r.getNumPositions();
coords = r.getStateFrame.getCoordinateNames();
coords = coords(1:nq);
l_leg_kny = find(strcmp(coords,'l_leg_kny'));
r_leg_kny = find(strcmp(coords,'r_leg_kny'));
l_leg_hpy = find(strcmp(coords,'l_leg_hpy'));
r_leg_hpy = find(strcmp(coords,'r_leg_hpy'));
l_leg_aky = find(strcmp(coords,'l_leg_aky'));
r_leg_aky = find(strcmp(coords,'r_leg_aky'));
l_leg_hpz = find(strcmp(coords,'l_leg_hpz'));
r_leg_hpz = find(strcmp(coords,'r_leg_hpz'));

q0 = nom_data.xstar(1:nq);
qdot0 = zeros(nq,1);
kinsol0 = doKinematics(r,q0,false,false);
r_foot_pos = forwardKin(r,kinsol0,r_foot,r_foot_pts,2);
l_foot_pos = forwardKin(r,kinsol0,l_foot,l_foot_pts,2);
r_hand_pos = forwardKin(r,kinsol0,r_hand,r_hand_pts,0);
l_hand_pos = forwardKin(r,kinsol0,l_hand,l_hand_pts,0);
com_pos0 = getCOM(r,kinsol0,1);
com_height = com_pos0(3);
tspan = [0,1];
kc1 = {WorldPositionConstraint(r,r_foot,r_foot_pts,r_foot_pos(1:3),r_foot_pos(1:3),tspan),...
  WorldQuatConstraint(r,r_foot,r_foot_pos(4:7),0,tspan)};
kc2 = {WorldPositionConstraint(r,l_foot,l_foot_pts,l_foot_pos(1:3),l_foot_pos(1:3),tspan),...
  WorldQuatConstraint(r,l_foot,l_foot_pos(4:7),0,tspan)};
kc3 = WorldPositionConstraint(r,r_hand,r_hand_pts,r_hand_pos+[0.1;0.05;0.75],r_hand_pos+[0.1;0.05;1],[tspan(end) tspan(end)]);
kc4 = WorldPositionConstraint(r,l_hand,l_hand_pts,l_hand_pos,l_hand_pos,[tspan(end) tspan(end)]);
kc5 = WorldCoMConstraint(r,[-inf;-inf;com_height],[inf;inf;com_height+0.5],tspan,1);
pc_knee = PostureConstraint(r,tspan);

pc_knee = pc_knee.setJointLimits([l_leg_kny;r_leg_kny],[0.2;0.2],[inf;inf]);

qsc = QuasiStaticConstraint(r);
qsc = qsc.addContact(r_foot,r_foot_contact_pts);
qsc = qsc.addContact(l_foot,l_foot_contact_pts);
qsc = qsc.setActive(true);
qsc = qsc.setShrinkFactor(0.9);

iAfun = [1;1;2;2;3;3;4;4];
jAvar = [l_leg_kny;r_leg_kny;l_leg_hpy;r_leg_hpy;l_leg_aky;r_leg_aky;l_leg_hpz;r_leg_hpz];
A = [1;-1;1;-1;1;-1;1;-1];
lb = [0;0;0;-0.1*pi];
ub = [0;0;0;0.1*pi];
stlpc = SingleTimeLinearPostureConstraint(r,iAfun,jAvar,A,lb,ub,tspan);

ikoptions = IKoptions(r);
cost = Point(r.getStateFrame,1);
cost.base_x = 100;
cost.base_y = 100;
cost.base_z = 100;
cost.base_roll = 100;
cost.base_pitch = 100;
cost.base_yaw = 100;
cost = double(cost);
Q = diag(cost(1:nq));
ikoptions = ikoptions.setQ(Q);
ikoptions = ikoptions.setQa(0.001*Q);
ikoptions = ikoptions.setMajorIterationsLimit(10000);
ikoptions = ikoptions.setIterationsLimit(500000);
ikoptions = ikoptions.setSuperbasicsLimit(1000);
ikoptions = ikoptions.setMajorOptimalityTolerance(2e-4);
ikoptions = ikoptions.setDebug(true);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
ikmexoptions = ikmexoptions.setDebug(true);
nT = 5;
% t = [tspan(1) tspan(1)+0.2*(tspan(end)-tspan(1)) tspan(1)+0.7*(tspan(end)-tspan(1)) tspan(end)];
t = linspace(tspan(1),tspan(end),nT);
q_nom_traj = PPTrajectory(foh(t,repmat(q0,1,nT)));
q_seed_traj = PPTrajectory(zoh(t,repmat(q0,1,nT)+[zeros(nq,1) 0*randn(nq,nT-1)]));

display('Check IK traj');
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,kc5,pc_knee,ikoptions);
v = r.constructVisualizer();
x_sol = xtraj.eval(t);
q_sol = x_sol(1:nq,:);
[robot_lb,robot_ub] = r.getJointLimits();
if(any(any(q_sol>bsxfun(@times,robot_ub,ones(1,nT)))) || any(any(q_sol<bsxfun(@times,robot_lb,ones(1,nT)))))
  error('posture solution is not within the joint limits');
end
if(any(any(isinf(x_sol))) || any(any(isnan(q_sol))))
  error('solution cannot be nan');
end
valuecheck(q_sol(:,1),q_seed_traj.eval(0),1e-3);
v.playback(xtraj,struct('slider',true));
display('Check IK traj with quasi static constraint');
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},qsc,kc2{:},kc3,kc4,kc5,pc_knee,ikoptions);
v = r.constructVisualizer();
v.playback(xtraj,struct('slider',true));
display('Check IK traj with SingleTimeLinearPostureConstraint')
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},qsc,kc2{:},kc3,kc4,kc5,pc_knee,stlpc,ikoptions);
v = r.constructVisualizer();
v.playback(xtraj,struct('slider',true));
x_breaks = xtraj.eval(t);
q_breaks = x_breaks(1:nq,:);
valuecheck(q_breaks([l_leg_hpy;l_leg_kny;l_leg_aky],2:end),q_breaks([r_leg_hpy;r_leg_kny;r_leg_aky],2:end),1e-10);
if(any(abs(q_breaks(l_leg_hpz,2:end)-q_breaks(r_leg_hpz,2:end))>0.1*pi+1e-8))
  error('SingleTimeLinearPostureConstraint is not satisfied');
end
display('Check IK traj with infeasibility')
kc_err = WorldCoMConstraint(r,[nan;nan;2],inf(3,1),tspan);
[xtraj,info,infeasible_constraint] = inverseKinTraj(r,t,q_seed_traj,q_nom_traj,kc_err,kc2{:},kc3,kc4,kc5,pc_knee,qsc,ikmexoptions);
if(info ~= 13)
  error('The problem should be infeasible');
end
display('The user should check that the infeasible constraints are the CoM z');
display('The infeasible constraints returned from IKtraj is');
display(infeasible_constraint);

% x0 = [q_seed_traj.eval(t(1));q_seed_traj.deriv(t(1))];
% ikproblem = InverseKinematicsTrajectory(r,t,q_nom_traj,ikoptions.fixInitialState,x0,kc_err,kc2{:},kc3,kc4,kc5,pc_knee,qsc);
% ikproblem = ikproblem.addBoundingBoxConstraint(BoundingBoxConstraint(ikoptions.qdf_lb,ikoptions.qdf_ub),ikproblem.qdf_idx);
% ikproblem = ikproblem.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_MajorIterationsLimit);
% ikproblem = ikproblem.setSolverOptions('snopt','IterationsLimit',ikoptions.SNOPT_IterationsLimit);
% % ikproblem = ikproblem.setSolverOptions('snopt','print','iktraj.out');
% [xtraj,F,info,infeasible_constraint] = ikproblem.solve(q_seed_traj);
% if(info ~= 13)
%   error('The problem should be infeasible');
% end

display('Check IK with WorldFixedPositionConstraint');
kc_fixedPosition = WorldFixedPositionConstraint(r,pelvis,[0;0;0],tspan);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,kc_fixedPosition,stlpc,ikoptions);

display('Check IK with WorldFixedOrientConstraint');
kc_fixedOrient = WorldFixedOrientConstraint(r,pelvis,tspan);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,kc_fixedOrient,stlpc,ikoptions);

display('Check IK with WorldFixedBodyPoseConstraint');
kc_fixedPose = WorldFixedBodyPoseConstraint(r,pelvis,tspan);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_knee,kc_fixedPose,stlpc,ikoptions);

display('Check IK with fixInitialState = false, the posture and velocity at the begining are also decision variables');
ikoptions = ikoptions.setFixInitialState(false);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_knee,kc_fixedPose,stlpc,ikoptions);

display('Check with inbetween samples')
t_inbetween = [0.1 0.15 0.3 0.4 0.6];
ikoptions = ikoptions.setAdditionaltSamples(t_inbetween);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_knee,kc_fixedPose,ikoptions);
t_samples = sort([t t_inbetween]);
x_samples = xtraj.eval(t_samples);
q_samples = x_samples(1:nq,:);
kinsol_samples = cell(1,size(q_samples,2));
for i = 1:size(q_samples,2)
  kinsol_samples{i} = r.doKinematics(q_samples(:,i),false,false);
end
c_fixedPose = kc_fixedPose.eval(t_samples,kinsol_samples);
[lb_fixedPose,ub_fixedPose] = kc_fixedPose.bounds(t_samples);
if(any(c_fixedPose-ub_fixedPose>1e-3) || any(c_fixedPose-lb_fixedPose<-1e-3))
  error('The fixed pose constraint is not satisfied');
end

ikoptions = ikoptions.setAdditionaltSamples([]);
display('Check MultipleKinematicConstraint with tspan being only part of the t_breaks');
tspan2 = [0.2 0.7];
kc_fixedPose = WorldFixedBodyPoseConstraint(r,pelvis,tspan2);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_knee,kc_fixedPose,ikoptions);

display('Check MultipleTimeLinearPostureConstraint');
pc_change = PostureChangeConstraint(r,[l_leg_kny;r_leg_kny],[-0.1;-0.1],[0.05;0.05],tspan);
ikoptions = ikoptions.setFixInitialState(true);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_change,ikoptions);
xbreaks = xtraj.eval(t);
if(any(xbreaks(l_leg_kny,3:end)-xbreaks(l_leg_kny,2)>0.05+1e-10) || any(xbreaks(l_leg_kny,3:end)-xbreaks(l_leg_kny,2)<-0.1-1e-10)||...
   any(xbreaks(r_leg_kny,3:end)-xbreaks(r_leg_kny,2)>0.05+1e-10) || any(xbreaks(r_leg_kny,3:end)-xbreaks(r_leg_kny,2)<-0.1-1e-10))
 error('PostureChangeConstraint is not satisfied');
end

ikoptions = ikoptions.setFixInitialState(false);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_change,ikoptions);
xbreaks = xtraj.eval(t);
if(any(xbreaks(l_leg_kny,2:end)-xbreaks(l_leg_kny,1)>0.05+1e-10) || any(xbreaks(l_leg_kny,2:end)-xbreaks(l_leg_kny,1)<-0.1-1e-10)||...
   any(xbreaks(r_leg_kny,2:end)-xbreaks(r_leg_kny,1)>0.05+1e-10) || any(xbreaks(r_leg_kny,2:end)-xbreaks(r_leg_kny,1)<-0.1-1e-10))
 error('PostureChangeConstraint is not satisfied');
end

display('Check MultipleTimeLinearPostureConstraint for time span being only part of the t_breaks');
pc_change2 = PostureChangeConstraint(r,[l_leg_kny;r_leg_kny],[-0.1;-0.1],[0.05;0.05],[t(2) t(end)]);
ikoptions = ikoptions.setFixInitialState(true);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_change2,ikoptions);
xbreaks = xtraj.eval(t(2:end));
if(any(xbreaks(l_leg_kny,2:end)-xbreaks(l_leg_kny,1)>0.05+1e-10) || any(xbreaks(l_leg_kny,2:end)-xbreaks(l_leg_kny,1)<-0.1-1e-10)||...
   any(xbreaks(r_leg_kny,2:end)-xbreaks(r_leg_kny,1)>0.05+1e-10) || any(xbreaks(r_leg_kny,2:end)-xbreaks(r_leg_kny,1)<-0.1-1e-10))
 error('PostureChangeConstraint is not satisfied');
end
ikoptions = ikoptions.setFixInitialState(false);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_change2,ikoptions);
xbreaks = xtraj.eval(t(2:end));
if(any(xbreaks(l_leg_kny,2:end)-xbreaks(l_leg_kny,1)>0.05+1e-10) || any(xbreaks(l_leg_kny,2:end)-xbreaks(l_leg_kny,1)<-0.1-1e-10)||...
   any(xbreaks(r_leg_kny,2:end)-xbreaks(r_leg_kny,1)>0.05+1e-10) || any(xbreaks(r_leg_kny,2:end)-xbreaks(r_leg_kny,1)<-0.1-1e-10))
 error('PostureChangeConstraint is not satisfied');
end

display('Compare using MultipleTimeLinearPostureConstraint and MultipleTimeKinematicConstraint');
ikoptions = ikoptions.setAdditionaltSamples([]);
lower_joints = cellfun(@(s) ~isempty(strfind(s,'leg')),coords);
joint_idx = (1:nq)';
lower_joints = joint_idx(lower_joints);
pc_change = PostureChangeConstraint(r,[(1:6)';lower_joints],zeros(6+length(lower_joints),1),zeros(6+length(lower_joints),1));
lfoot_onground = WorldPositionConstraint(r,l_foot,l_foot_contact_pts,[nan(2,size(l_foot_contact_pts,2));zeros(1,size(l_foot_contact_pts,2))],[nan(2,size(l_foot_contact_pts,2));zeros(1,size(l_foot_contact_pts,2))]);
rfoot_onground = WorldPositionConstraint(r,r_foot,r_foot_contact_pts,[nan(2,size(l_foot_contact_pts,2));zeros(1,size(r_foot_contact_pts,2))],[nan(2,size(r_foot_contact_pts,2));zeros(1,size(r_foot_contact_pts,2))]);
lhand_cnst = WorldPositionConstraint(r,l_hand,l_hand_pts,[1;2;1],[1;2;2]);
ikoptions = ikoptions.setFixInitialState(false);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,lfoot_onground,rfoot_onground,lhand_cnst,pc_change,qsc,ikoptions);
xbreaks = xtraj.eval(t);
if(any(any(abs(diff(xbreaks([(1:6)';lower_joints],:),[],2))>1e-4)))
  error('PostureChangeConstraint is not satisfied');
end
lfoot_fixed = WorldFixedBodyPoseConstraint(r,l_foot);
rfoot_fixed = WorldFixedBodyPoseConstraint(r,r_foot);
pelvis_fixed = WorldFixedBodyPoseConstraint(r,pelvis);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,lfoot_fixed,rfoot_fixed,pelvis_fixed,lhand_cnst,qsc,ikoptions);
xbreaks = xtraj.eval(t);

end

function xtraj = test_IKtraj_userfun(r,t,q_seed,q_nom,varargin)
ikoptions = varargin{end};
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
ikoptions = ikoptions.setMex(false);
display('IK mex start to solve the problem');
tic
[xtraj,info,infeasible_constraint] = inverseKinTraj(r,t,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
if(info>10)
  error('SNOPT info is %d, inverseKinTraj mex fails to solve the problem',info);
end
toc
x_sol = xtraj.eval(t);
q_sol = x_sol(1:r.getNumPositions,:);
testConstraint(r,t,q_sol,ikoptions.fixInitialState,varargin{1:end-1});
tic
x0 = [q_seed.eval(t(1));q_seed.deriv(t(1))];
ikproblem = InverseKinematicsTrajectory(r,t,q_nom,ikoptions.fixInitialState,x0,varargin{1:end-1});
ikproblem = ikproblem.addBoundingBoxConstraint(BoundingBoxConstraint(ikoptions.qdf_lb,ikoptions.qdf_ub),ikproblem.qdf_idx);
ikproblem = ikproblem.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_MajorIterationsLimit);
ikproblem = ikproblem.setSolverOptions('snopt','IterationsLimit',ikoptions.SNOPT_IterationsLimit);
ikproblem = ikproblem.setSolverOptions('snopt','MajorOptimalityTolerance',ikoptions.SNOPT_MajorOptimalityTolerance);
% ikproblem = ikproblem.setSolverOptions('snopt','print','iktraj.out');
[xtraj,F,info,infeasible_constraint] = ikproblem.solve(q_seed);
if(info>10)
  error('SNOPT info is %d, InverseKinematicsTrajectory fails to solve the problem',info);
end
toc
x_sol = xtraj.eval(t);
q_sol = x_sol(1:r.getNumPositions,:);
testConstraint(r,t,q_sol,ikoptions.fixInitialState,varargin{1:end-1});
% display('IK matlab start to solve the problem');
% tic
% [xtraj,info,infeasible_constraint] = inverseKinTraj(r,t,q_seed,q_nom,varargin{1:end-1},ikoptions);
% toc
% if(info>10)
%   error('SNOPT info is %d, IK fails to solve the problem',info);
% end
end

function testConstraint(robot,t,q_sol,fix_initial_state,varargin)
nT = length(t);
if(fix_initial_state)
  t_start_idx = 2;
  nT_free = nT-1;
else
  t_start_idx = 1;
  nT_free = nT;
end

kinsol = cell(1,nT);
for i = t_start_idx:length(t)
  kinsol{i} = robot.doKinematics(q_sol(:,i),false,false);
end
[q_lb,q_ub] = robot.getJointLimits();
% The interpolation seems to change the value of q at the final knot point by epsilon
if(any(any(q_sol(:,t_start_idx:end-1)>bsxfun(@times,q_ub,ones(1,nT_free-1))))...
    || any(any(q_sol(:,t_start_idx:end-1)<bsxfun(@times,q_lb,ones(1,nT_free-1)))))
  error('solution must be within the robot default joint limits');
end
if(any(q_sol(:,end)-q_ub>3*eps) || any(q_sol(:,end)-q_lb<-3*eps))
  error('solution must be within the robot default joint limits');
end
for i = 1:nargin-4
  if(isa(varargin{i},'SingleTimeKinematicConstraint'))
    for j = t_start_idx:nT
      if(varargin{i}.isTimeValid(t(j)))
        [lb,ub] = varargin{i}.bounds(t(j));
        c = varargin{i}.eval(t(j),kinsol{j});
        if(any(c-ub>1e-3) || any(c-lb<-1e-3))
          error('solution does not satisfy the SingleTimeKinematicConstraint');
        end
      end
    end
  elseif(isa(varargin{i},'PostureConstraint'))
    for j = t_start_idx:nT
      if(varargin{i}.isTimeValid(t(j)))
        [lb,ub] = varargin{i}.bounds(t(j));
        if(any(q_sol(:,j)>ub+1e-10) || any(q_sol(:,j)<lb-1e-10))
          error('solution does not satisfy the PostureConstraint');
        end
      end
    end
  elseif(isa(varargin{i},'QuasiStaticConstraint'))
    for j = t_start_idx:nT
      if(varargin{i}.isTimeValid(t(j)) && varargin{i}.active)
        if(~varargin{i}.checkConstraint(kinsol{j}))
          error('solution does not satisfy the QuasiStaticConstraint');
        end
      end
    end
  elseif(isa(varargin{i},'SingleTimeLinearPostureConstraint'))
    for j = t_start_idx:nT
      if(varargin{i}.isTimeValid(t(j)))
        [lb,ub] = varargin{i}.bounds(t(j));
        c = varargin{i}.feval(t(j),q_sol(:,j));
        if(any(c-ub>1e-3) || any(c-lb < -1e-3))
          error('solution does not satisfy the SingleTimeLinearPostureConstraint');
        end
      end
    end
  elseif(isa(varargin{i},'MultipleTimeKinematicConstraint'))
    [lb,ub] = varargin{i}.bounds(t(t_start_idx:end));
    c = varargin{i}.eval(t(t_start_idx:end),kinsol(t_start_idx:end));
    if(any(c-ub>1e-3) || any(c-lb < -1e-3))
      error('solution does not satisfy the MultipleTimeKinematicConstraint');
    end
  elseif(isa(varargin{i},'MultipleTimeLinearPostureConstraint'))
    [lb,ub] = varargin{i}.bounds(t(t_start_idx:end));
    c = varargin{i}.feval(t(t_start_idx:end),q_sol(:,t_start_idx:end));
    if(any(c-ub>1e-3) || any(c-lb < -1e-3))
      error('solution does not satisfy the MultipleTimeLinearPostureConstraint');
    end
  else
    error('The constraint is not supported');
  end
end
end
