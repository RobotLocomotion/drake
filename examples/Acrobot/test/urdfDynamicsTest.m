function urdfDynamicsTest

oldpath=addpath('..');
p_orig = AcrobotPlant;
p_urdf_2D = PlanarRigidBodyManipulator('../Acrobot.urdf');
p_urdf_3D = RigidBodyManipulator('../Acrobot.urdf');

for i=1:25
  t = rand;
  x = randn(4,1);
  u = randn;
  
  xdot = p_orig.dynamics(t,x,u);
  valuecheck(p_urdf_2D.dynamics(t,x,u), xdot);
  valuecheck(p_urdf_3D.dynamics(t,x,u), xdot);

end

path(oldpath);
