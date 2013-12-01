function trigPolyTest

options.replace_output_w_new_state = true;

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
p1 = PlanarRigidBodyManipulator('../Pendulum.urdf');
warning(w);

tp1 = extractTrigPolySystem(p1,options);

oldpath=addpath('..');
p2 = PendulumPlant();
path(oldpath);

tp2 = extractTrigPolySystem(p2,options);

% test numerically, because f and e are different (one has the inertial
% matrix in f, the other has it in e)
for i=1:25
  x = Point(p1.getStateFrame,randn(2,1));
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

  xnp(1) = mod(xnp(1),2*pi);
  xn(1) = mod(xn(1),2*pi);
  valuecheck(xn,xnp,1e-2);
end
