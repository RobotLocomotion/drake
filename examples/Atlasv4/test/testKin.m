function testKin

options.floating = true;
%options.dt = 0.001;
r = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);

tic
for i=1:100
  q = randn(34,1); qd = randn(34,1);
  kinsol = doKinematics(r,q,true,true);
%  [x,J,dJ] = forwardKin(r,kinsol,body_ind,[0;0;0]);
%  Jdot = matGradMult(reshape(dJ,3*nq,nq),qd);
end
toc

tic
for i=1:100
  q = randn(34,1); qd = randn(34,1);
  kinsol = doKinematics(r,q,false,true,qd);
%  Jdot = forwardJacDot(r,kinsol,body_ind,[0;0;0]);
end
toc

