function urdfDynamicsTest

oldpath=addpath(fullfile(pwd,'..'));
p_orig = PendulumPlant;
p_urdf_2D = PlanarRigidBodyManipulator('../Pendulum.urdf');
p_urdf_3D = RigidBodyManipulator('../Pendulum.urdf');

[~,U_zero_orig] = energy(p_orig,zeros(2,1));
[~,U_zero_2D] = energy(p_urdf_2D,zeros(2,1));
[~,U_zero_3D] = energy(p_urdf_3D,zeros(2,1));

for i=1:25
  t = rand;
  x = randn(2,1);
  u = randn;
    
  xdot = p_orig.dynamics(t,x,u);
  valuecheck(p_urdf_2D.dynamics(t,x,u), xdot);
  valuecheck(p_urdf_3D.dynamics(t,x,u), xdot);

  [T,U] = p_orig.energy(x);
  [T_test,U_test] = p_urdf_2D.energy(x);
  valuecheck(T_test,T);
  valuecheck(U_test-U_zero_2D,U-U_zero_orig);
  [T_test,U_test] = p_urdf_3D.energy(x);
  valuecheck(T_test,T);
  valuecheck(U_test-U_zero_3D,U-U_zero_orig);
end

path(oldpath);
