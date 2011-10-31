function urdfDynamicsTest

oldpath=addpath('..');
p_orig = AcrobotPlant;
p_urdf = PlanarURDFManipulator('../Acrobot.urdf');

for i=1:25
  t = rand;
  x = randn(4,1);
  u = randn;
  
  [H1,C1,B1]=p_urdf.manipulatorDynamics(x(1:2),x(3:4));
  [H2,C2,B2]=p_orig.manipulatorDynamics(x(1:2),x(3:4));
  
  xdoterr = p_urdf.dynamics(t,x,u) - p_orig.dynamics(t,x,u)
  if (any(abs(xdoterr)>1e-5))
    [H1,H2]
    [C1,C2]
    [B1,B2]
    error('dynamics don''t match');
  end
end

path(oldpath);
