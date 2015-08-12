function testFixedContactsSearchQ1
checkDependency('spotless');
addpath(fullfile(pwd,'..'));
num_contacts = 4;
num_fc_edges = 4;
mu_face = 1;
p = SynthesizeGraspSphere(0.04,[0;0;0],num_contacts,mu_face,struct('lin_fc_flag',true,'num_fc_edges',num_fc_edges));
solver_sol = p.optimize();
[sol,sol_bilinear] = p.retrieveSolution(solver_sol);
p.plotSolution(sol,sol_bilinear);
Qw = diag([10;10;10;200;200;200]);
p1 = FixedContactsSearchQ1LinFC(zeros(3,1),num_contacts,num_fc_edges,Qw);
s = 0.04;
m_L0_mat = p1.findL0Mat(sol.fc);
[solver_sol1,info,solver_time] = p1.searchQ1(m_L0_mat,sol.fc);
sol1 = p1.retrieveSolution(solver_sol1);
radius_true = computeQ1LinFC(sol.contact_pos,[0;0;0],sol.fc_edges,Qw);
display(sprintf('SOS verifies Q1: %10.7f',sol1.radius));
display(sprintf('      Actual Q1: %10.7f',radius_true));
end