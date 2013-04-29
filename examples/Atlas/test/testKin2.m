function testKin2

options.floating = true;
r = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);
q=zeros(getNumDOF(r),1);
pelvis = findLinkInd(r,'pelvis');

for i=1:100
  q(1) = randn();
  q(2) = randn();
  q(3) = randn();
  q(4) = 2*pi*(rand()-0.5);
  q(5) = 2*pi*(rand()-0.5);
  q(6) = 2*pi*(rand()-0.5);
  kinsol = doKinematics(r,q,false,true);
  xyzrpy = forwardKin(r,kinsol,pelvis,[0;0;0],1);
  valuecheck(xyzrpy,q(1:6));
end
