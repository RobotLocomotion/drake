function urdfDynamicsTest

oldpath=addpath(fullfile(pwd,'..'));
p_orig = PendulumPlant;
p_urdf_2D = PlanarRigidBodyManipulator('../Pendulum.urdf');
p_urdf_3D = RigidBodyManipulator('../Pendulum.urdf');

for i=1:25
  t = rand;
  x = randn(2,1);
  u = randn;
    
  xdot = p_orig.dynamics(t,x,u);
  valuecheck(p_urdf_2D.dynamics(t,x,u), xdot);
  valuecheck(p_urdf_3D.dynamics(t,x,u), xdot);
end

path(oldpath);
