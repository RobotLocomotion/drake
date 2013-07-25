function trigPolyTest

options.replace_output_w_new_state = true;

p1 = PlanarRigidBodyManipulator('../Acrobot.urdf');
tp1 = makeTrigPolySystem(p1,options);

oldpath=addpath('..');
p2 = AcrobotPlant();
path(oldpath);

tp2 = makeTrigPolySystem(p2,options);

% test numerically, because f and e are different (one has the inertial
% matrix in f, the other has it in e)
for i=1:25
  x = Point(p1.getStateFrame,randn(4,1));
  xp = x.inFrame(tp1.getStateFrame);
  x = double(x); xp=double(xp);
  u = randn;

  xdot = p1.dynamics(0,x,u);
  xdotp = tp1.dynamics(0,xp,u);
  valuecheck(xdot,p2.dynamics(0,x,u));
  valuecheck(xdotp,tp2.dynamics(0,xp,u));
  
  dt = .1;
  xn = x+dt*xdot;
  xnp = Point(tp1.getStateFrame,xp+dt*xdotp);
  xnp = double(xnp.inFrame(p1.getStateFrame));
  valuecheck(xn,xnp,1e-2);
end
