function urdfDynamicsTest

oldpath=addpath('..');
p_orig = PendulumPlant;
p_urdf = PlanarRigidBodyManipulator('../Pendulum.urdf');

for i=1:25
  t = rand;
  x = randn(2,1);
  u = randn;
    
  xdoterr = p_urdf.dynamics(t,x,u) - p_orig.dynamics(t,x,u);
  if (any(abs(xdoterr)>1e-5))
    t
    x
    u
    p_urdf.dynamics(t,x,u)
    p_orig.dynamics(t,x,u)
    error('dynamics don''t match');
  end
end

path(oldpath);
