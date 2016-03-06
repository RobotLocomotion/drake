function [sol,sol_bilinear,info,solver_time] = testOptimalEllipseGraspLinFC(A_ellipse,b_ellipse)
% NOTEST
checkDependency('spotless');
addpath(fullfile(pwd,'..'));
if(nargin<1)
  quat = randn(4,1);
  quat = quat/norm(quat);
  A_ellipse = diag([0.1;0.03;0.02])*quat2rotmat(quat);
  b_ellipse = zeros(3,1);
end
num_contacts = 4;
num_fc_edges = 4;
mu_face = 1;
Qw = diag([10;10;10;200;200;200]);

prog = OptimalEllipseGraspLinFC(A_ellipse,b_ellipse,b_ellipse,num_contacts,mu_face,num_fc_edges,Qw);
prog.iter_max = 10;
prog.optimal_tol = -5e-2;
prog.initial_grasp_prog.final_backoff_flag = true;
prog.contact_step.final_backoff_flag = true;
[sol,sol_bilinear,info,solver_time] = prog.findOptimalGrasp();
end
