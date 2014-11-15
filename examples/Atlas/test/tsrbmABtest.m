function tsrbmABtest

r = TimeSteppingRigidBodyManipulator('../urdf/robotiq.urdf',.001);
r2 = TimeSteppingRBMUpdate('../urdf/robotiq.urdf',.001);

nx = getNumStates(r);
nu = getNumInputs(r);
for i=1:100
  [x,success] = resolveConstraints(r,randn(nx,1));
  x = double(x);
  u = randn(nu,1);
  if success
    xn = update(r,0,x,u);
    xn2 = update(r2,0,x,u);
    valuecheck(xn,xn2);
  end
end