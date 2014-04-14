% clear all
options.terrain = RigidBodyFlatTerrain();
p = TimeSteppingRigidBodyManipulator('snake.urdf',.01,options);
p = p.addRobotFromURDF('snake.urdf',[.4*randn(2,1);0],0*randn(3,1));
% p = p.addRobotFromURDF('snake.urdf',[.4*randn(2,1);0],0*randn(3,1));
% p = p.addRobotFromURDF('snake.urdf',[.4*randn(2,1);0],0*randn(3,1));
% p = p.addRobotFromURDF('snake.urdf',.4*randn(3,1),randn(3,1));
% p = p.addRobotFromURDF('snake.urdf',.5*randn(3,1),randn(3,1));
x0 = 2*randn(p.getNumDiscStates,1);
v = p.constructVisualizer();
v.drawWrapper(0,x0);
x0 = p.resolveConstraints(x0);
v.drawWrapper(0,x0);
x0 = double(x0);
x0(floor(length(x0)/2)+1:end) = 0;
x0
traj = simulate(p,[0 5],x0);
v.playback(traj)

%%
t = traj.tt;
x = traj.eval(t);
nq = p.getManipulator.getNumPositions;
M = p.getMass;
for i=1:length(t),
  q = x(1:nq,i);
  qd = x(nq+1:end,i);
  kinsol = doKinematics(p,q);
  [H,C,B] = manipulatorDynamics(p,q,qd);
  T(i) = .5*qd'*H*qd;
  com = p.getCOM(kinsol);
  U(i) = com(3)*M*9.81;
  T2(i) = .5*qd(7:end)'*H(7:end,7:end)*qd(7:end);
end
