function testKin2

options.floating = true;
r = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);
q=zeros(getNumPositions(r),1);
pelvis = findLinkId(r,'pelvis');

for i=1:100
  q(1) = randn();
  q(2) = randn();
  q(3) = randn();
  q(4) = 2*pi*(rand()-0.5);
  q(5) = 2*pi*(rand()-0.5);
  q(6) = 2*pi*(rand()-0.5);
  kinsol = doKinematics(r,q,false,false);
  xyzrpy = forwardKin(r,kinsol,pelvis,[0;0;0],1);
  if ~valuecheck(rpy2quat(xyzrpy(4:6)),rpy2quat(q(4:6)))
    valuecheck(-rpy2quat(xyzrpy(4:6)),rpy2quat(q(4:6)));
  end
end
