function testKin

options.floating = true;
%options.dt = 0.001;
r = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);

tic
for i=1:100
  q = randn(r.num_positions,1); qd = randn(r.num_velocities,1);
  kinsol = doKinematics(r,q,true,true);
%  [x,J,dJ] = forwardKin(r,kinsol,body_ind,[0;0;0]);
end
toc

tic
for i=1:100
  q = randn(r.num_positions,1); qd = randn(r.num_velocities,1);
  kinsol = doKinematics(r,q,false,true,qd);
end
toc

