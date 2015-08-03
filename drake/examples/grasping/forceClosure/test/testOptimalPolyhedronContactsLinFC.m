function [sol,sol_bilinear] = testOptimalPolyhedronContactsLinFC(verts)
% NOTEST
checkDependency('spotless');
addpath(fullfile(pwd,'..'));
if(nargin<1)
  verts = 0.04*randn(3,1000);
end
num_contacts = 4;
num_fc_edges = 4;
mu_face = 1;
shrink_factor = 0.8;
Qw = diag([10;10;10;250;250;250]);

prog = OptimalPolyhedronGraspLinFC(verts,zeros(3,1),num_contacts,mu_face,num_fc_edges,shrink_factor,Qw);
prog.iter_max = 50;
prog.optimal_tol = -3e-2;
prog.contact_step.res_tol = 2e-3;
prog.lagrangian_step.backoff_flag = true;
prog.contact_step.final_backoff_flag = true;
[sol,sol_bilinear] = prog.findOptimalGrasp();
end
