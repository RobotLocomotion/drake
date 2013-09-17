function trigPolyTest

options.replace_output_w_new_state = true;

p1 = PlanarRigidBodyManipulator('../Acrobot.urdf');
tp1 = extractTrigPolySystem(p1,options);

oldpath=addpath('..');
p2 = AcrobotPlant();
path(oldpath);

tp2 = extractTrigPolySystem(p2,options);

w = warning('off','Drake:DrakeSystem:ConstraintsNotEnforced');
tf = .5;
% test numerically, because f and e are different (one has the inertial
% matrix in f, the other has it in e)
for i=1:5
  x = Point(p1.getStateFrame,randn(4,1));
  xp = x.inFrame(tp1.getStateFrame);
  x = double(x); xp=double(xp);
  u = randn;

  xdot = p1.dynamics(0,x,u);
  xdotp = tp1.dynamics(0,xp,u);
  valuecheck(xdot,p2.dynamics(0,x,u));
  valuecheck(xdotp,tp2.dynamics(0,xp,u));
  
  xtraj = simulate(p1,[0 tf],x);
  xptraj = simulate(tp1,[0 tf],xp);
  
  valuecheck(xtraj.eval(tf),double(Point(tp1.getStateFrame,xptraj.eval(tf)).inFrame(p1.getStateFrame)),1e-2);
end

warning(w);
