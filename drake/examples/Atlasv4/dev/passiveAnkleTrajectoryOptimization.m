function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = passiveAnkleTrajectoryOptimization(z0,xtraj,utraj,ltraj)
% [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = passiveAnkleTrajectoryOptimization(z0,xtraj,utraj,ltraj)
% Arguments are optional initial seeds for the optimization program
% If one argument (z0) is supplied, this is the seed vector
% If multiple arguments are supplied, xtraj, utraj, and ltraj are initial
% trajectories for the optimization. z0 is the recommended method for
% running sequential optimizations.

% Construct plant and visualizer objects
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;

p = PlanarRigidBodyManipulator('../urdf/atlas_simple_spring_ankle_planar_contact.urdf',options);
p= p.setInputLimits(max(p.umin,-60),min(p.umax,60));
nq = p.getNumPositions;
nv = p.getNumVelocities;
nu = p.getNumInputs;
nx = p.getNumStates();
v = p.constructVisualizer(struct('viewer','BotVisualizer'));

% Number of knot points per hybrid mode
N = [7,4,7];

% Duration bounds on each mode
duration = {[.1 1],[.05 .4], [.1 1]};

% Active contact sequence in each mode
% 1-left heel
% 2-left toe
% 3-right heel
% 4-right toe
modes = {[1;2],[2;3;4], [3;4]};

% Minimum average speed in m/s
speed_min = .7;

% % Maximum total time
% T_max = 1;


% Nominal initial state selected from contact implicit optimization
x0 = [      0
    0.9371
    0.2000
   -0.4414
    0.2625
   -0.0211
    0.0891
   -0.4997
    0.9273
   -0.6403
    0.2346
   -0.0077
    0.0731
   -0.2012
    0.7876
   -0.6596
    0.1798
   -1.9375
    2.5602
   -2.2319]; 
 
% Subset of indices needed for constraints
% Original indices are ordered [x;z] per contact
% The removed index is the redundant x direction at the toe when the full
% foot is on the ground
to_options.mode_options{1}.active_inds = [1;2;4];
to_options.mode_options{2}.active_inds = [1;2;4;5;6];
to_options.mode_options{3}.active_inds = [1;2;4];

% small regularization term
to_options.mode_options{1}.accel_cost = .0001;
to_options.mode_options{2}.accel_cost = .0001;
to_options.mode_options{3}.accel_cost = .0001;

traj_opt = ConstrainedHybridTrajectoryOptimization(p,modes,N,duration,to_options);

% Manually generated constraint on the average speed
% avg_speed = xf/T >= speed_min
%  ==>   xf - T*speed_min >= 0
h_inds = [traj_opt.mode_opt{1}.h_inds + traj_opt.var_offset(1);...
  traj_opt.mode_opt{2}.h_inds + traj_opt.var_offset(2);...
  traj_opt.mode_opt{3}.h_inds + traj_opt.var_offset(3)];
xf_ind = traj_opt.var_offset(3) + traj_opt.mode_opt{end}.x_inds(1,end); % Index of x position at final time

traj_opt = traj_opt.addConstraint(LinearConstraint(0,inf,[1 -speed_min*ones(1,numel(h_inds))]),[xf_ind;h_inds]);

% Add foot height constraint
[~,normal,d,xA,xB,idxA,idxB,mu,n,D] = contactConstraints(p,x0(1:p.getNumPositions));
if idxA(1) == 1,
  % Swap A and B in this case
  tmp = idxA;
  idxA = idxB;
  idxB = tmp;
  tmp = xA;
  xA = xB;
  xB = tmp;
end
assert(isequal(idxB,ones(size(idxB))))
 
% Swing leg heel height
fn1 = drakeFunction.kinematic.WorldPosition(p,idxA(3),p.T_2D_to_3D'*xA(:,3),2);

% Swing leg toe height
fn2 = drakeFunction.kinematic.WorldPosition(p,idxA(4),p.T_2D_to_3D'*xA(:,4),2);

% Enforce both toe and heel more than 2cm above ground during swing

for i=1:N(1)-3,
  traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.02,inf,p.getNumPositions, @fn1.eval, 1), i, 1:p.getNumPositions);
  traj_opt = traj_opt.addModeStateConstraint(1,FunctionHandleConstraint(.02,inf,p.getNumPositions, @fn2.eval, 1), i, 1:p.getNumPositions);
end

% Toe height of swing in mode 3, enforcing minimum height of 1cm and 2cm
% along the mode
fn3 = drakeFunction.kinematic.WorldPosition(p,idxA(2),p.T_2D_to_3D'*xA(:,2),2);
traj_opt = traj_opt.addModeStateConstraint(3,FunctionHandleConstraint(.01,inf,p.getNumPositions, @fn3.eval, 1), 2, 1:p.getNumPositions);
for i=3:N(3)
traj_opt = traj_opt.addModeStateConstraint(3,FunctionHandleConstraint(.02,inf,p.getNumPositions, @fn3.eval, 1), i, 1:p.getNumPositions);
end

% Rough nominal weight distribution between heel and toe. Generated from
% static standing pose
l0 = [0;897.3515;0;179.1489];

% Initial state constraint, it's very loose (1 radian on joints and 10
% rad/s on velocities)
traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [0;1*ones(9,1);10*ones(10,1)],x0+[0;1*ones(9,1);10*ones(10,1)]),1);

% Minimum step length of 45cm--deleted in favor of average speed constraint
% traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(.45,inf),N(end),1);

% Generate initial guesses using l0 for contact forces, x0 for states, and
% random control inputs
t_init{1} = linspace(0,.4,N(1));
traj_init.mode{1}.x = PPTrajectory(foh(t_init{1},repmat(x0,1,N(1))));
traj_init.mode{1}.u = PPTrajectory(foh(t_init{1},randn(5,N(1))));
traj_init.mode{1}.l = PPTrajectory(foh(t_init{1},repmat(l0,1,N(1))));

if length(N) > 1
  t_init{2} = linspace(0,.2,N(2));
  traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},repmat(x0,1,N(2))));
  traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(5,N(2))));
  traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},repmat([l0;l0(1:2)],1,N(2))));
end
if length(N) > 2
  t_init{3} = linspace(0,.2,N(3));
  traj_init.mode{3}.x = PPTrajectory(foh(t_init{3},repmat(x0,1,N(3))));
  traj_init.mode{3}.u = PPTrajectory(foh(t_init{3},randn(5,N(3))));
  traj_init.mode{3}.l = PPTrajectory(foh(t_init{3},repmat([l0],1,N(3))));
  
  % build periodic constraint matrix representing mirrored periodicity for
  % half stride
  R_periodic = zeros(p.getNumStates,2*p.getNumStates);
  R_periodic(2,2) = 1; %z
  R_periodic(3,3) = 1; %pitch
  R_periodic(4:6,8:10) = eye(3); %leg joints w/symmetry
  R_periodic(8:10,4:6) = eye(3); %leg joints w/symmetry
  R_periodic(7,7) = 1; % back joint
  R_periodic(11:13,11:13) = eye(3); %x,z,pitch velocities
  R_periodic(14:16,18:20) = eye(3); %leg joints w/symmetry
  R_periodic(18:20,14:16) = eye(3); %leg joints w/symmetry
  R_periodic(17,17) = 1; % back joint
  R_periodic(2:end,p.getNumStates+2:end) = -eye(p.getNumStates-1);
  
  periodic_constraint = LinearConstraint(zeros(nx-1,1),zeros(nx-1,1),R_periodic(2:end,:));
  periodic_inds = [traj_opt.mode_opt{1}.x_inds(:,1) + traj_opt.var_offset(1);...
    traj_opt.mode_opt{end}.x_inds(:,end) + traj_opt.var_offset(end)];
  traj_opt = traj_opt.addConstraint(periodic_constraint,periodic_inds);
end

% If initial trajectories were supplied, use them instead
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

% Set SNOPT iteration limits and output file
traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
% 1000 is a lot of iterations! Reduce if the program is stalling
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',50000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',2000000);

% Add Add cost functions on pelvis motion and control inputs
for i=1:length(N)
  traj_opt = traj_opt.addModeRunningCost(i,@running_cost_fun);
  traj_opt = traj_opt.addModeRunningCost(i,@pelvis_motion_fun);
  
end

for i=1:length(N)
  % add joint limits as constraints
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(p.joint_limit_min,p.joint_limit_max),1:N(i),1:10);
  
  % bound joint velocities at +/- 4 rad/s
  joint_vel_max = 4;
  joint_vel_bound = BoundingBoxConstraint(-joint_vel_max*ones(p.getNumVelocities,1),joint_vel_max*ones(p.getNumVelocities,1));
  traj_opt = traj_opt.addModeStateConstraint(i,joint_vel_bound,1:N(i),11:20);
end

traj_opt = traj_opt.compile();
if nargin == 1
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTrajFromZ(z0);
else
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
end



  function [f,df] = running_cost_fun(h,x,u)
    % cost on control input
    R = .1;
    f = h*u'*R*u;
    df = [u'*R*u zeros(1,20) 2*h*u'*R];
  end

  function [f,df] = pelvis_motion_fun(h,x,u)
    % Cost on pelvis pitch and velocity
    nu = length(u);  
    pitch_idx = 3;
    pitch_dot_idx = p.getNumPositions+pitch_idx;

    Kq = 500;
    Kqd = 500;
    f = Kq*x(pitch_idx)^2 + Kqd*x(pitch_dot_idx)^2;
    df = zeros(1,1+nx+nu);
    df(1+pitch_idx) = 2*Kq*x(pitch_idx);
    df(1+pitch_dot_idx) = 2*Kqd*x(pitch_dot_idx);
    
    df = df*h;
    df(1) = f;
    f = f*h;
  end
end