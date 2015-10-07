function [p,xtraj,utraj,ltraj,z,F,info,traj_opt] = periodicTrajOpt(z)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
options.use_new_kinsol = true;
p = RigidBodyManipulator('OneLegHopper.urdf',options);
v = p.constructVisualizer();

%todo: add joint limits, periodicity constraint

N = [7,7 7 7];
duration = {[.01 .3], [.01 .5], [.05 .2], [.05 .5]};
modes = {[1;2;3;4], [3;4], [], [1;2;3;4]};

distance = .1;
x_vel = -0.0;

qd_init = [x_vel;zeros(7,1)];

q0 = [0;0;0;0;.6;0;-1.2;.6+pi/2];
phi_f = p.contactConstraints(q0);
q0(3) = -phi_f(1);
x0 = [q0;qd_init];

q1 = [-distance/2;0;0;0;.6;0;-1.2;.2+pi/2];
phi_f = p.contactConstraints(q1);
q1(3) = -phi_f(1) + 0.15;
x1 = [q1;qd_init];

qf = [-distance;0;0;0;.6;0;-1.2;.6+pi/2];
phi_f = p.contactConstraints(qf);
qf(3) = -phi_f(1);
xf = [qf;qd_init];

% tf0 = 0.5;
% if nargin < 2  
%   t_init = linspace(0,tf0,N);
%   traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,floor(N/2)),linspacevec(x1,xf,N-floor(N/2))]));
%   traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame);
%  
%   traj_init.u = PPTrajectory(foh(t_init,zeros(getNumInputs(p),N)));
% end

for i=1:length(N),
  t_init{i} = linspace(0,.1,N(i));
  traj_init.mode{i}.x = PPTrajectory(foh(t_init{i},repmat(x0,1,N(i))));
  traj_init.mode{i}.u = PPTrajectory(foh(t_init{i},randn(2,N(3))));
end


x0_min = [q0;qd_init];
x0_max = [q0;qd_init];

xf_min = [qf;qd_init] - [.0;zeros(15,1)];
xf_max = [qf;qd_init] + [.0;zeros(15,1)];

to_options = struct();

% to_options.mode_options{1}.active_inds = [1;2;4];
% to_options.mode_options{2}.active_inds = [1;2];
% to_options.mode_options{3}.active_inds = [];
% to_options.mode_options{4}.active_inds = [1;2;4];

% to_options.integration_method = ContactImplicitTrajectoryOptimization.MIDPOINT;
% to_options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;
% to_options.integration_method = ContactImplicitTrajectoryOptimization.BACKWARD_EULER;

traj_opt = ConstrainedHybridTrajectoryOptimization(p,modes,N,duration,to_options);

for i=1:length(N),
  traj_opt = traj_opt.addModeRunningCost(i,@running_cost_fun);
  traj_opt = traj_opt.addModeRunningCost(i,@foot_height_fun);
  traj_opt = traj_opt.addModeFinalCost(i,@final_cost_fun);
end


traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(xf_min,xf_max),N(end));

% traj_opt = traj_opt.addInputConstraint(LinearConstraint(zeros(2,1),zeros(2,1),[eye(2),-eye(2)]),{[1,N-1]});% force first and next-to-last inputs to be equal (we drop the last input because it's junk)

% traj_opt = traj_opt.setCheckGrad(true);
traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',500);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',5000);
traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-5);

if nargin == 1
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTrajFromZ(z);
else
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
end

  function [f,df] = contact_delta_cost_fun(l)
    K = 500;
    f = K*sum(diff(l).^2);
    n = length(l);
    R = sparse([1:n-1 2:n], [1:n-1 1:n-1], [ones(n-1,1);-ones(n-1,1)]);
    df = -2*K*diff(l)'*R';
  end

  function [f,df] = running_cost_fun(h,x,u)
    K = 10;
    R = eye(getNumInputs(p));
    f = K*u'*R*u;
    df = [0 zeros(1,16) 2*K*u'*R];
  end

  function [f,df] = foot_height_fun(h,x,u)
    q = x(1:8);
    K = 5000;
    [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',false));
    phi0 = .1*ones(size(phi));
    f = K*(phi - phi0)'*(phi - phi0);
    % phi: 2x1
    % n: 2xnq
    df = [0 2*K*(phi-phi0)'*n zeros(1,10)];
  end

  function [f,df] = final_cost_fun(T,x)
    K = 1000;
    f = K*T;
    df = [K zeros(1,16)];
    
  end

end
