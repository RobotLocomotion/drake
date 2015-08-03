function [sol,sol_bilinear,info,solver_time] = testOptimalSphereContactsLinFC()
% NOTEST
checkDependency('spotless');
addpath(fullfile(pwd,'..'));
num_contacts = 3;
num_fc_edges = 4;
mu_face = 1;
sphere_radius = 0.04;
sphere_center = [0;0;0];

Qw = diag([10;10;10;1000;1000;1000]);
prog = OptimalSphereGraspLinFC(sphere_radius,sphere_center,sphere_center,num_contacts,mu_face,num_fc_edges,Qw);
prog.iter_max = 100;
prog.optimal_tol = -1e-2;
prog.lagrangian_step.backoff_flag = false;
% prog.lagrangian_step.backoff_scale = 0.9;
% prog.contact_step.final_backoff_flag = true;
[sol,sol_bilinear,info,solver_time] = prog.findOptimalGrasp();
end
