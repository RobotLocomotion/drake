function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = test3D3ModeHybridConstrainedDircol(z0,xtraj,utraj,ltraj)

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;

p = RigidBodyManipulator('../urdf/atlas_simple_contact.urdf',options);
nq = p.getNumPositions;
nv = p.getNumVelocities;
nu = p.getNumInputs;
nx = p.getNumStates();
v = p.constructVisualizer();

N = [6,3,4];

duration = {[.2 .7],[.05 .4], [.1 .7]};

modes = {[1;2;3;4],[3;4;5;6;7;8],[5;6;7;8]};

% duration = duration(1:3);
% modes = modes(1:3);
% N = N(1:3);


x0 = [      0   % x
    0           % y
    0.9371      % z
    0           % roll
    0.2000      % pitch
    0           % yaw
    0           % lhpz
    0           % lhpx
   -0.4414      % lhpy
    0.2625      % lkny
   -0.0211      % laky
    0           % lakx
    0           % bkz
    0.0891      % bky
    0           % rhpz
    0           % rhpx
   -0.4997      % rhpy
    0.9273      % rkny
   -0.6403      % raky
    0           % rakx
    0           % bkx
    0.2346
    0           % y
   -0.0077
    0           % roll
    0.0731
    0           % yaw
    0           % lhpz
    0           % lhpx
   -0.2012
    0.7876
   -0.6596
    0           % lkax
    0           % bkx
    0.1798
    0           % rhpz
    0           % rhpx
   -1.9375
    2.5602
   -2.2319
    0           % rakx
    0           % bkx
   ]; 
 init3DTraj = load('data/transformed3DNonfeasible');
  
%  x0 = init3DTraj.threeDTraj.xtraj{1}.eval(0);

 to_options = struct();
to_options.mode_options{1}.active_inds = [1;2;3;4;6;9];
to_options.mode_options{2}.active_inds = [1;2;3;4;6;7;8;9;10;12;15];
to_options.mode_options{3}.active_inds = [1;2;3;4;6;9];

% to_options.mode_options{1}.friction_limits = false;
% to_options.mode_options{2}.friction_limits = false;
% to_options.mode_options{3}.friction_limits = false;

traj_opt = ConstrainedHybridTrajectoryOptimization(p,modes,N,duration,to_options);


% Add foot height constraint
[~,normal,d,xA,xB,idxA,idxB,mu,n,D] = contactConstraints(p,x0(1:p.getNumPositions));
if idxA(1) == 1,
  tmp = idxA;
  idxA = idxB;
  idxB = tmp;
  tmp = xA;
  xA = xB;
  xB = tmp;
end
assert(isequal(idxB,ones(size(idxB))))
 
fn1 = drakeFunction.kinematic.WorldPosition(p,idxA(5),xA(:,5),3);
fn2 = drakeFunction.kinematic.WorldPosition(p,idxA(7),xA(:,7),3);
% traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.1,inf,p.getNumPositions, @fn1.eval, 1), floor(N(1)/2), 1:p.getNumPositions);
% traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.1,inf,p.getNumPositions, @fn2.eval, 1), floor(N(1)/2), 1:p.getNumPositions);

for i=floor(N(1)/2)+1:N(1)-1,
% traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.01,inf,p.getNumPositions, @fn1.eval, 1), i, 1:p.getNumPositions);
% traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.05,inf,p.getNumPositions, @fn2.eval, 1), i, 1:p.getNumPositions);  
end


l0 = [0;0;897.3515;0;0;897.3515;0;0;179.1489;0;0;179.1489]/2;

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [0;.1*ones(20,1);.1*ones(21,1)],x0+[0;.1*ones(20,1);.1*ones(21,1)]),1);
% traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [0;.02*ones(20,1);.1*ones(21,1)],x0+[0;.02*ones(20,1);.1*ones(21,1)]),1);
traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(.3,inf),N(end),1);

t_init{1} = linspace(0,.4,N(1));
traj_init.mode{1}.x = PPTrajectory(foh(t_init{1},repmat(x0,1,N(1))));
traj_init.mode{1}.u = PPTrajectory(foh(t_init{1},randn(15,N(1))));
traj_init.mode{1}.l = PPTrajectory(foh(t_init{1},repmat(l0,1,N(1))));

if length(N) > 1
  t_init{2} = linspace(0,.2,N(2));
  traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},repmat(x0,1,N(2))));
  traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(15,N(2))));
  traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},repmat([l0;l0(1:6)],1,N(2))));
end
% if length(N) > 2
%   t_init{3} = linspace(0,.2,N(3));
%   traj_init.mode{3}.x = PPTrajectory(foh(t_init{3},repmat(x0,1,N(3))));
%   traj_init.mode{3}.u = PPTrajectory(foh(t_init{3},randn(15,N(3))));
%   traj_init.mode{3}.l = PPTrajectory(foh(t_init{3},repmat([l0;l0],1,N(3))));
% end
if length(N) > 2
  t_init{3} = linspace(0,.2,N(3));
  traj_init.mode{3}.x = PPTrajectory(foh(t_init{3},repmat(x0,1,N(3))));
  traj_init.mode{3}.u = PPTrajectory(foh(t_init{3},randn(15,N(3))));
  traj_init.mode{3}.l = PPTrajectory(foh(t_init{3},repmat([l0],1,N(3))));
  
  % build periodic constraint matrix

  R_periodic = zeros(p.getNumStates,2*p.getNumStates);
  R_periodic(2,2) = -1; %y
  R_periodic(3,3) = 1; %z
  R_periodic(4,4) = -1; %roll
  R_periodic(5,5) = 1; %pitch
  R_periodic(6,6) = -1; %yaw
  R_periodic(7:12,15:20) = diag([-1;-1;1;1;1;-1]); %leg joints w/symmetry
  R_periodic(15:20,7:12) = diag([-1;-1;1;1;1;-1]); %leg joints w/symmetry
  R_periodic(13,13) = -1; %bkz
  R_periodic(14,14) = 1; %bky
  R_periodic(21,21) = -1; %bkx
  R_periodic(22:42,22:42) = R_periodic(1:21,1:21);
  R_periodic(22,22) = 1; % xvel  
  R_periodic(2:end,p.getNumStates+2:end) = -eye(p.getNumStates-1);
  
  periodic_constraint = LinearConstraint(zeros(nx-1,1),zeros(nx-1,1),R_periodic(2:end,:));
  periodic_inds = [traj_opt.mode_opt{1}.x_inds(:,1) + traj_opt.var_offset(1);...
    traj_opt.mode_opt{end}.x_inds(:,end) + traj_opt.var_offset(end)];
  traj_opt = traj_opt.addConstraint(periodic_constraint,periodic_inds);
end

if nargin > 1
  for i=1:length(N)
    t_init{i} = linspace(0,diff(xtraj{i}.tspan),N(i));
    traj_init.mode{i}.x = xtraj{i};
    if nargin < 2
      traj_init.mode{i}.u = utraj{i};
    end
    if nargin > 3
      traj_init.mode{i}.l = ltraj{i};
    end
  end
end

if true && nargin < 1  
  
  for i=1:length(N),
    t_init{i} = init3DTraj.threeDTraj.xtraj{i}.pp.breaks;
    if length(t_init{i}) ~= N(i)
      t_init{i} = linspace(0,t_init{i}(end),N(i));
    end
    traj_init.mode{i}.x = init3DTraj.threeDTraj.xtraj{i};
    traj_init.mode{i}.u = init3DTraj.threeDTraj.utraj{i};
    traj_init.mode{i}.l = init3DTraj.threeDTraj.ltraj{i};
  end
end

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt2.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',500);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',50000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',2000000);



% traj_opt = traj_opt.addModeRunningCost(1,@foot_height_fun);
% traj_opt = traj_opt.setCheckGrad(true);
for i=1:length(N)
  traj_opt = traj_opt.addModeRunningCost(i,@running_cost_fun);
  traj_opt = traj_opt.addModeRunningCost(i,@pelvis_motion_fun);
  
end
% traj_opt = traj_opt.addModeRunningCost(2,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(3,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(4,@running_cost_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);

for i=1:length(N)
%   knee_inds = traj_opt.mode_opt{i}.x_inds([5;9],:) + traj_opt.var_offset(i);
%   knee_inds = knee_inds(:);
%   knee_constraint = BoundingBoxConstraint(.1*ones(length(knee_inds),1),inf(length(knee_inds),1));
%   traj_opt = traj_opt.addBoundingBoxConstraint(knee_constraint,knee_inds);
  
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(p.joint_limit_min,p.joint_limit_max),1:N(i),1:p.getNumPositions);
  
  % bound joint velocities
  joint_vel_max = 10;
  joint_vel_bound = BoundingBoxConstraint(-joint_vel_max*ones(p.getNumVelocities,1),joint_vel_max*ones(p.getNumVelocities,1));
  traj_opt = traj_opt.addModeStateConstraint(i,joint_vel_bound,1:N(i),p.getNumPositions+1:nx);
  
  
  % bound out of plane positions
  outplane_max = .2;
  outplane_inds = find(cellfun(@any,(regexp(p.getStateFrame.coordinates(1:21),'(base_y|_..x|_..z|roll|yaw)$'))));
  outplane_bound = BoundingBoxConstraint(-outplane_max*ones(length(outplane_inds),1),outplane_max*ones(length(outplane_inds),1));
  traj_opt = traj_opt.addModeStateConstraint(i,outplane_bound,1:N(i),outplane_inds);
end

traj_opt = traj_opt.compile();
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',traj_opt.num_vars+1);
if nargin == 1
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTrajFromZ(z0);
else
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
end


  function [f,df] = running_cost_fun(h,x,u)
    R_diag = 1*ones(15,1);
    %     R_diag([1;3;5;6;7;8;11;12;13;14]) = 10*ones(10,1); %aky,akx
    R_diag([1;3;5;7;9;11;13;15]) = 10*ones(8,1);
    R = diag(R_diag);
    f = h*u'*R*u;
    df = [u'*R*u zeros(1,numel(x)) 2*h*u'*R];
  end

  function [f,df] = pelvis_motion_fun(h,x,u)
    nu = length(u);  
    idx = [2;3;4;5;6];
    dot_idx = p.getNumPositions+idx;

    Kq = 5;
    Kqd = 50;
    f = Kq*x(idx)'*x(idx) + Kqd*x(dot_idx)'*x(dot_idx); % try to avoid moving the pelvis quickly
    df = zeros(1,1+nx+nu);
    df(1+idx) = 2*Kq*x(idx)';
    df(1+dot_idx) = 2*Kqd*x(dot_idx)';
    
    df = df*h;
    df(1) = f;
    f = f*h;
    
%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end

  function [f,df] = foot_height_fun(h,x,u)
    q = x(1:nq);
    
    [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',true));
    phi0 = .2*ones(8,1);
    K = 500;
    
%     [~,I1] = min(phi(1:2));
%     [~,I2] = min(phi(3:4));
%     phi = [phi(I1);phi(2+I2)];
%     n = [n(I1,:);n(2+I2,:)];    
%     phi0 = [.2;.2];
    
    I = find(phi < phi0);
    f = K*(phi(I) - phi0(I))'*(phi(I) - phi0(I));
    % phi: 2x1
    % n: 2xnq
    df = [0 2*K*(phi(I)-phi0(I))'*n(I,:) zeros(1,nv+nu)];
    
    df = df*h;
    df(1) = f;
    f = f*h;
    
    
%     K = 500;
%     f = K*sum(phi0(I) - phi(I));
%     df = [0 K*sum(n(I,:),1) zeros(1,nv+nu)];

%    K = 100;
%    K_log = 100;
%    f = sum(-K*log(K_log*phi + .2));
%    df = [0 sum(-K*K_log*n./(K_log*repmat(phi,1,length(q)) + .2)) zeros(1,15)];
  end

end