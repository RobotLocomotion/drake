function urdfDynamicsTest(xtest,utest)

oldpath=addpath(fullfile(pwd,'..'));
p_orig = AcrobotPlant;
p_urdf_2D = PlanarRigidBodyManipulator('../Acrobot.urdf');
p_urdf_3D = RigidBodyManipulator('../Acrobot.urdf');

[~,U_zero_orig] = energy(p_orig,zeros(4,1));
[~,U_zero_2D] = energy(p_urdf_2D,zeros(4,1));
[~,U_zero_3D] = energy(p_urdf_3D,zeros(4,1));

for i=1:25
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

  [T,U] = p_orig.energy(x);
  [T_test,U_test] = p_urdf_2D.energy(x);
  valuecheck(T_test,T);
  valuecheck(U_test-U_zero_2D,U-U_zero_orig);
  [T_test,U_test] = p_urdf_3D.energy(x);
  valuecheck(T_test,T);
  valuecheck(U_test-U_zero_3D,U-U_zero_orig);
end

path(oldpath);
