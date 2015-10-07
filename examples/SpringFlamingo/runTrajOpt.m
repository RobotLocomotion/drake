function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = runTrajOpt(xtraj,utraj,ltraj,ljltraj,scale)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
p = PlanarRigidBodyManipulator('spring_flamingo_nobase.urdf',options);
% trajopt = ContactImplicitTrajectoryOptimization(p,[],[],[],10,[1 1]);

%todo: add joint limits, periodicity constraint

N = 15;

% periodic constraint
R_periodic = zeros(p.getNumStates,2*p.getNumStates);
R_periodic(2,2) = 1; %z 
R_periodic(3,3) = 1; %pitch
R_periodic(4:6,7:9) = eye(3); %leg joints w/symmetry
R_periodic(7:9,4:6) = eye(3); %leg joints w/symmetry

R_periodic(10:12,10:12) = eye(3); %x,z,pitch velocities
R_periodic(13:15,16:18) = eye(3); %leg joints w/symmetry
R_periodic(16:18,13:15) = eye(3); %leg joints w/symmetry

R_periodic(2:end,p.getNumStates+2:end) = -eye(p.getNumStates-1);

periodic_constraint = LinearConstraint(zeros(p.getNumStates,1),zeros(p.getNumStates,1),R_periodic);

%red to blue
q0 = [0; .875; 0; 0;0;0;0;0;0];
x0 = [q0;zeros(9,1)];
% xf = x0 + [.6;zeros(17,1)];



N2 = floor(N/2);

if nargin < 2
  
  t0 = .65;
  tf = 1.65;  
  traj_jerry = load('../jerry.mat');
  t_init = linspace(0,tf-t0,N);
  x = traj_jerry.traj.eval(t_init+t0);
  x = x([1 2 3 7 9 11 13 15 17 4 5 6 8 10 12 14 16 18],:);
  x(1,:) = x(1,:) - x(1,1);
%   x(4:9,:) = -x(4:9,:);
%   x(13:18,:) = -x(13:18,:);
  traj_init.x = PPTrajectory(foh(t_init,x));
  traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame);
  tf0 = tf-t0;
%   load('../full_input');
%  utraj0 = PPTrajectory(foh(t,interp1(t_input,u_input,t+t0)'));
  traj_init.u = PPTrajectory(foh(linspace(0,tf0,N),randn(6,N)));

  N1 = floor(N/2);
  N2 = N-N1;
%   %Try to come up with a reasonable trajectory
%   x1 = [-.2;.84;pi/16;-pi/4;pi/8;zeros(13,1)]
%   tf0 = 2;
%   xvec = x0*ones(1,N1) + (x1-x0)*linspace(0,1,N1);
%   xvec = [xvec, x1*ones(1,N2) + (xf-x1)*linspace(0,1,N2)];
%   xtraj0.x = PPTrajectory(foh(linspace(0,tf0,N),xvec));
  d = floor(N/4);
  lp = [1;0;0;0];
  ln = zeros(4,1);
  traj_init.l = PPTrajectory(foh(t_init,[repmat([ln;ln;lp;lp],1,N1-d) zeros(16,2*d) repmat([lp;lp;ln;ln],1,N2-d)]));
  traj_init.ljl = [];
  
  scale = 1;
else
  t_init = xtraj.pp.breaks;
  traj_init.x = xtraj;
  traj_init.u = utraj;
  traj_init.l = ltraj;
  traj_init.ljl = ljltraj;
  
  tf0 = t_init(end);
end
T_span = [.8 1.5];


x0_min = [x0(1); x0(2); -.1; -inf(4,1); -0; -inf; -5; -.1; -5*ones(7,1)];
x0_max = [x0(1); x0(2);  .1;  inf(4,1);  0;  inf;  5;  .1;  5*ones(7,1)];

xf_min = [-inf(18,1)];
xf_max = [-.57;inf(17,1)];  %walking negative x direction

to_options.compl_slack = scale*.01;
to_options.lincompl_slack = scale*.001;
to_options.jlcompl_slack = scale*.01;

to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.lambda_mult = p.getMass*9.81*tf0/N/2;
to_options.lambda_jl_mult = tf0/N;

to_options.integration_method = ContactImplicitTrajectoryOptimization.MIDPOINT;

traj_opt = ContactImplicitTrajectoryOptimization(p,N,T_span,to_options);
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
% traj_opt = traj_opt.addRunningCost(@foot_height_fun);
traj_opt = traj_opt.addFinalCost(@final_cost_fun);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
traj_opt = traj_opt.addStateConstraint(periodic_constraint,{[1 N]});


% l1 = traj_opt.l_inds(5:end,1:5);
% l2 = traj_opt.l_inds(1:4,end-4:end);
% traj_opt = traj_opt.addConstraint(ConstantConstraint(0*l1(:)),l1);
% traj_opt = traj_opt.addConstraint(ConstantConstraint(0*l2(:)),l2);
% traj_opt = traj_opt.setCheckGrad(true);
snprint('snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',5000);
[xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);

function [f,df] = running_cost_fun(h,x,u)
  K = 100;
  R = eye(6);
  R(3,3) = 100;
  R(6,6) = 100;
  f = K*u'*R*u;
  df = [0 zeros(1,18) 2*K*u'*R];
end

function [f,df] = foot_height_fun(h,x,u)
  q = x(1:6);
  K = 1000;
  [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',false));
  phi0 = [.1;.1];
  f = K*(phi - phi0)'*(phi - phi0);
  % phi: 2x1
  % n: 2xnq
  df = [0 2*K*(phi-phi0)'*n zeros(1,9)];
end

function [f,df] = final_cost_fun(T,x)
  K = 1;
  f = K*T;
  df = [K zeros(1,18)];
end

end
