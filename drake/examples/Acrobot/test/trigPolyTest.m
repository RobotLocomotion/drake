function trigPolyTest

options.replace_output_w_new_state = true;

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
p1 = PlanarRigidBodyManipulator('../Acrobot_no_collision.urdf');
warning(w);
tp1 = extractTrigPolySystem(p1,options);

oldpath=addpath(fullfile(pwd,'..'));
p2 = AcrobotPlant();

tp2 = extractTrigPolySystem(p2,options);

w = warning('off','Drake:DrakeSystem:ConstraintsNotEnforced');

tf = findTransform(tp1.getStateFrame,p1.getStateFrame);

% test numerically, because f and e are different (one has the inertial
% matrix in f, the other has it in e)
for i=1:25
  x = Point(p1.getStateFrame,randn(4,1));
  xp = x.inFrame(tp1.getStateFrame);
  x = double(x); xp=double(xp);
  x(1:2) = mod(x(1:2),2*pi);
  u = randn;

  xdot = p1.dynamics(0,x,u);
  xpdot = tp1.dynamics(0,xp,u);
  valuecheck(xdot,p2.dynamics(0,x,u));
  valuecheck(xpdot,tp2.dynamics(0,xp,u));
  
  [xpx,dxdxp] = geval(@tf.output,[],[],xp);
  xpx(1:2) = mod(xpx(1:2),2*pi);
  valuecheck(x,xpx);
  valuecheck(xdot,dxdxp*xpdot);
end

warning(w);
path(oldpath);


function xnp = euler(dt)
  xnp = Point(tp1.getStateFrame,xp+dt*xpdot);
  xnp = double(xnp.inFrame(p1.getStateFrame));
end

end