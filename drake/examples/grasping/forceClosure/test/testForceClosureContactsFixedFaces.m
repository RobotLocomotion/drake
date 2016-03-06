function testForceClosureContactsFixedFaces()
checkDependency('spotless');
addpath(fullfile(pwd,'..'));
verts = repmat([0.025;0.03;0.02],1,8).*[1 1 1 1 -1 -1 -1 -1;1 1 -1 -1 1 1 -1 -1;1 -1 1 -1 1 -1 1 -1];
face_idx = [1,2,3,4];
mu_face = 2;
num_contacts = 4;
shrink_factor = 0.8;
p = SynthesizeGraspFixedFaces(verts,num_contacts,face_idx,mu_face,shrink_factor);
solver_sol = p.optimize();
[sol,sol_bilinear] = p.retrieveSolution(solver_sol);
p.use_lcmgl = true;
p.plotSolution(sol,sol_bilinear);
p.use_lcmgl = false;
p.plotSolution(sol,sol_bilinear);

p = SynthesizeGraspFixedFacesLinFriction(verts,num_contacts,face_idx,mu_face,4,shrink_factor);
solver_sol = p.optimize();
[sol,sol_bilinear] = p.retrieveSolution(solver_sol);
p.use_lcmgl = true;
p.plotSolution(sol,sol_bilinear);
p.use_lcmgl = false;
p.plotSolution(sol,sol_bilinear);
end