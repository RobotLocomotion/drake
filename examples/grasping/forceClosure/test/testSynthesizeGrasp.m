function testSynthesizeGrasp
% test the force closure grasp for different geometries

% display('test SynthesizeGraspFixedFaces on a small cuboid');
% verts = repmat([0.025;0.03;0.02],1,8).*[1 1 1 1 -1 -1 -1 -1;1 1 -1 -1 1 1 -1 -1;1 -1 1 -1 1 -1 1 -1];
% p = SynthesizeGraspFixedFaces(verts,3,[1 3 4],1,0.8);
% test_userfun(p);
% 
% display('test SynthesizeGraspFixedFacesLinFriction on a small cuboid');
% verts = repmat([0.025;0.03;0.02],1,8).*[1 1 1 1 -1 -1 -1 -1;1 1 -1 -1 1 1 -1 -1;1 -1 1 -1 1 -1 1 -1];
% p = SynthesizeGraspFixedFacesLinFriction(verts,3,[1 3 4],1,4,0.8);
% test_userfun(p);
% 
checkDependency('spotless');
addpath(fullfile(pwd,'..'));
display('test SynthesizeGraspSphere on a small sphere');
p = SynthesizeGraspSphere(0.02,100+randn(3,1),3,1);
test_userfun(p);
p = SynthesizeGraspSphere(0.02,randn(3,1),3,1,struct('lin_fc_flag',true,'num_fc_edges',4));
test_userfun(p);

display('test SynthesizeGraspSphere on a large sphere');
p = SynthesizeGraspSphere(1,randn(3,1),3,1);
test_userfun(p);
p = SynthesizeGraspSphere(1,randn(3,1),3,1,struct('lin_fc_flag',true,'num_fc_edges',4));
test_userfun(p);

display('test SynthesizeGraspEllipse on a small ellipse')
A_xc = diag([1+rand();2+rand();4+rand()]);
quat_rand = randn(4,1);
quat_rand = quat_rand/norm(quat_rand);
A_xc = rotmatFromQuatBilinear(quat_rand*quat_rand')*A_xc;
p = SynthesizeGraspEllipse(0.01*A_xc,randn(3,1),3,1);
test_userfun(p);
p = SynthesizeGraspEllipse(0.01*A_xc,randn(3,1),3,1,struct('lin_fc_flag',true,'num_fc_edges',4));
test_userfun(p);

display('test SynthesizeGraspEllipse on a large ellipse')
p = SynthesizeGraspEllipse(A_xc,randn(3,1),3,1);
test_userfun(p);
p = SynthesizeGraspEllipse(A_xc,randn(3,1),3,1,struct('lin_fc_flag',true,'num_fc_edges',4));
test_userfun(p);

display('test SynthesizeGraspCylinderSide on a small cylinder');
quat = randn(4,1);quat = quat/norm(quat);
b_cylinder = 100+randn(3,1);
p = SynthesizeGraspCylinderSide(quat,b_cylinder,0.03,0.1,3,1);
solver_sol = test_userfun(p);
p = SynthesizeGraspCylinderSide(quat,b_cylinder,0.03,0.1,3,1,struct('lin_fc_flag',true,'num_fc_edges',4));
solver_sol = test_userfun(p);

display('test SynthesizeGraspCylinderSide on a large cylinder');
quat = randn(4,1);quat = quat/norm(quat);
p = SynthesizeGraspCylinderSide(quat,randn(3,1),1,1,3,1);
test_userfun(p);
p = SynthesizeGraspCylinderSide(quat,randn(3,1),1,1,3,1,struct('lin_fc_flag',true,'num_fc_edges',4));
test_userfun(p);

display('test SynthesizeGraspFreeFaces on small cuboid');
verts = repmat([0.05;0.02;0.03],1,8).*[1 1 1 1 -1 -1 -1 -1;1 1 -1 -1 1 1 -1 -1;1 -1 1 -1 1 -1 1 -1];
p = SynthesizeGraspFreeFaces(verts,3,1,0.8);
test_userfun(p);
p = SynthesizeGraspFreeFaces(verts,3,1,0.8,struct('lin_fc_flag',true,'num_fc_edges',4));
test_userfun(p);

display('test SynthesizeGraspFreeFaces on large cuboid');
verts = repmat([5;2;3],1,8).*[1 1 1 1 -1 -1 -1 -1;1 1 -1 -1 1 1 -1 -1;1 -1 1 -1 1 -1 1 -1];
p = SynthesizeGraspFreeFaces(verts,3,1,0.8);
test_userfun(p);
p = SynthesizeGraspFreeFaces(verts,3,1,0.8,struct('lin_fc_flag',true,'num_fc_edges',4));
test_userfun(p);

display('test SynthesizeGraspFreeFaces on a random small polytope')
verts = 0.02*randn(3,1000);
p = SynthesizeGraspFreeFaces(verts,3,1,0.8);
test_userfun(p);
p = SynthesizeGraspFreeFaces(verts,3,1,0.8,struct('lin_fc_flag',true,'num_fc_edges',4));
test_userfun(p);

display('test SynthesizeGraspFreeFaces on a random large polytope')
verts = randn(3,1000);
p = SynthesizeGraspFreeFaces(verts,3,1,0.8);
test_userfun(p);
p = SynthesizeGraspFreeFaces(verts,3,1,0.8,struct('lin_fc_flag',true,'num_fc_edges',4));
test_userfun(p);
end

function solver_sol = test_userfun(p)
[solver_sol,info,itr,solver_time] = p.optimize();
if(info~=1)
  error('Info = %d,Failed to find a force closure grasp',info);
end
[sol,sol_bilinear] = p.retrieveSolution(solver_sol);
G = graspTransferMatrix(sol.contact_pos);
if(p.epsilonG<=0)
  error('epsilonG should be positive');
end
if(norm(G*sol.f(:))>1e-3 || min(eig(G*G'))<eps || any(sum(sol.f.^2,1)<1e-10))
  error('Not force closure');
end
if(p.lin_fc_flag)
  for i = 1:p.num_contacts
    if(norm(sol.fc_rotmat{i}*sol.fc_rotmat{i}'-eye(3))>1e-3 || abs(det(sol.fc_rotmat{i})-1)>1e-3)
      error('fc_rotmat{%d} is not a rotation matrix',i);
    end
    if(norm(sol.fc_rotmat{i}\sol.fc_edges{i}-p.fc_edges0)>1e-3)
      error('fc_edges{%d} is not rotated from fc_edges0 by fc_rotmat{%d}',i,i);
    end
  end
end
p.use_lcmgl = true;
p.plotSolution(sol,sol_bilinear);
p.use_lcmgl = false;
p.plotSolution(sol,sol_bilinear);
end