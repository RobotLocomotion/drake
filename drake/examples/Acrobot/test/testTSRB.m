function testTSRB

% unit test to make sure that TimeSteppingRigidBodyManipulator works for
% the case when there are no contacts, limits, etc.

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
dt = .001;
r = TimeSteppingRigidBodyManipulator('../Acrobot.urdf',dt);
r_orig = PlanarRigidBodyManipulator('../Acrobot.urdf');
r_orig = setSimulinkParam(r_orig,'FixedStep',num2str(dt),'Solver','ode1');
warning(w);


x0 = randn(4,1);
xtraj = simulate(r,[0 4],x0);
xtraj_orig = simulate(r_orig,[0 4],x0);

v = r_orig.constructVisualizer();
v2 = v;
v2.fade_percent = .5;
mv = MultiVisualizer({v2,v});

mv.playback([setOutputFrame(xtraj,getOutputFrame(xtraj_orig));xtraj_orig]);

% this fails for the (albiet rare) trajectories that approach an unstable
% fixed point, presumably due to differences in forward vs backward euler.
%valuecheck(eval(xtraj(1:2),4),eval(xtraj_orig(1:2),4),.1);

for i=1:40
  t = randn(); x = randn(4,1); u = randn();
  xn = update(r,t,x,u);
%  xn_orig = x + dt*dynamics(r_orig,t,x,u);
  % match backward euler update 
  xdot = dynamics(r_orig,t,x,u);
  qdn = x(3:4) + dt*xdot(3:4);
  qn = x(1:2) + dt*qdn;
  xn_orig = [qn;qdn];
  valuecheck(xn,xn_orig);
end

