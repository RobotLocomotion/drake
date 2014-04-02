% clear all
p = TimeSteppingRigidBodyManipulator('snake.urdf',.01);
p = p.addRobotFromURDF('ground_plane.urdf');
p = p.addRobotFromURDF('snake.urdf',[.4*randn(2,1);0],0*randn(3,1));
p = p.addRobotFromURDF('snake.urdf',[.4*randn(2,1);0],0*randn(3,1));
p = p.addRobotFromURDF('snake.urdf',[.4*randn(2,1);0],0*randn(3,1));
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