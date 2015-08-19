function urdfDynamicsTest(xtest,utest)

oldpath=addpath('..');
p_orig = UnderwaterAcrobotPlant;
p_urdf_2D = PlanarRigidBodyManipulator('../UnderwaterAcrobot.urdf');
p_urdf_3D = RigidBodyManipulator('../UnderwaterAcrobot.urdf');

for i=1:25
    disp(['Trial ' mat2str(i)]);
  t = rand;
  if nargin>0 && i<=size(xtest,2)
    x = xtest(:,i);
  else
    x = randn(4,1);
  end
  if nargin>1 && i<=size(utest,2)
    u = utest(:,i);
  else
    u = randn;
  end  
  xdot = p_orig.dynamics(t,x,u);
  valuecheck(p_urdf_2D.dynamics(t,x,u), xdot);
  valuecheck(p_urdf_3D.dynamics(t,x,u), xdot);

end

path(oldpath);
