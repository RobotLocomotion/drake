function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = hybridTrajOpt(z)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;
p = PlanarRigidBodyManipulator('../KneedCompassGait.urdf',options);
p = p.setJointLimits(-inf(6,1),inf(6,1));
p = p.compile();
v = p.constructVisualizer();
% trajopt = ContactImplicitTrajectoryOptimization(p,[],[],[],10,[1 1]);

%todo: add joint limits, periodicity constraint

% 5/5,.1 S
% 5/5,.01 F
% 10/10,1 Success
% 15/15,1 and 10 Fail
% 15/15,20 S
% 20/20,50 F
% 20/20,70 S

N = [14, 15];
T = {[.2 1], [.2 1]};
mode_seq = {1, 2};

% periodic constraint
R_periodic = zeros(p.getNumStates,2*p.getNumStates);
R_periodic(2,2) = 1; %z
R_periodic(3,3) = 1; %pitch-hip w/symmetry
R_periodic(3,5) = 1; %pitch-hip w/symmetry
R_periodic(4,6) = 1; %knee w/symmetry
R_periodic(6,4) = 1; %knee w/symmetry
R_periodic(5,5) = -1; %hip w/symmetry

R_periodic(7,7) = 1; %x-vel
R_periodic(8,8) = 1; %z-vel
R_periodic(9,9) = 1; %pitch-hip w/symmetry
R_periodic(9,11) = 1; %pitch-hip w/symmetry
R_periodic(10,12) = 1; %knee w/symmetry
R_periodic(12,10) = 1; %knee w/symmetry
R_periodic(11,11) = -1; %hip w/symmetry

R_periodic(2:end,p.getNumStates+2:end) = -eye(p.getNumStates-1);

periodic_constraint = LinearConstraint(zeros(p.getNumStates,1),zeros(p.getNumStates,1),R_periodic);

% x0 = [0;0;1;zeros(15,1)];
% xf = [0;0;1;zeros(15,1)];
x0 = [0;1;zeros(10,1)];
xf = [.2;1;zeros(10,1)];

x1 = [.1;1;pi/8-pi/16;pi/8;-pi/8;pi/8;zeros(6,1)];
N2 = floor(sum(N)/2);

W = p.getMass*9.8;

%Try to come up with a reasonable trajectory
t_init{1} = linspace(0,1,N(1));
traj_init.mode{1}.x = PPTrajectory(foh(t_init{1},linspacevec(x0,x1,N(1))));
traj_init.mode{1}.u = PPTrajectory(foh(t_init{1},randn(3,N(1))));
traj_init.mode{1}.l = PPTrajectory(foh(t_init{1},repmat([0;W],1,N(1))));

% t_init{2} = linspace(0,.25,N(2));
% traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},linspacevec(x1,x1,N(2))));
% traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(3,N(2))));
% traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},repmat([0;W/2;0;W/2],1,N(2))));

t_init{2} = linspace(0,1,N(2));
traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},linspacevec(x1,xf,N(2))));
traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(3,N(2))));
traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},repmat([0;W],1,N(2))));

  
  

x0_min = [x0(1:5);-inf; -inf; 0; -inf(4,1)];
x0_max = [x0(1:5);inf;  inf; 0; inf(4,1)];
xf_min = [.4;-inf(11,1)];
xf_max = inf(12,1);

to_options.lambda_bound = 20;

traj_opt=ConstrainedHybridTrajectoryOptimization(p,mode_seq,N,T,to_options);

for i=1:length(N),
  traj_opt = traj_opt.addModeRunningCost(i,@running_cost_fun);
  traj_opt = traj_opt.addModeRunningCost(i,@foot_height_fun);
  
  % bound velocities to [-3,3]
  traj_opt = traj_opt.addModeStateConstraint(i,BoundingBoxConstraint(-3*ones(6,1),3*ones(6,1)),1:N(i),7:12);
  traj_opt = traj_opt.addModeFinalCost(i,@final_cost_fun);
end


traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addModeStateConstraint(length(N),BoundingBoxConstraint(xf_min,xf_max),N(end));



periodic_inds = [traj_opt.mode_opt{1}.x_inds(:,1) + traj_opt.var_offset(1);...
  traj_opt.mode_opt{end}.x_inds(:,end) + traj_opt.var_offset(end)];
traj_opt = traj_opt.addConstraint(periodic_constraint,periodic_inds);

traj_opt = traj_opt.setSolverOptions('snopt','ScaleOption',1);
traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',400);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',400000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',300000);
traj_opt = traj_opt.compile();

if nargin == 1
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTrajFromZ(z);
else
  [xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
end

function [f,df] = running_cost_fun(h,x,u)
  f = u'*u;
  df = [0 zeros(1,12) 2*u'];
end

function [f,df] = foot_height_fun(h,x,u)
  q = x(1:6);
  K = 50;
  [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',false));
  phi0 = [.1;.1];
  f = K*(phi - phi0)'*(phi - phi0);
  % phi: 2x1
  % n: 2xnq
  df = [0 2*K*(phi-phi0)'*n zeros(1,9)];
end

function [f,df] = final_cost_fun(T,x)
  K = 500;
  f = K*T;
  df = [K zeros(1,12)];
end

end
