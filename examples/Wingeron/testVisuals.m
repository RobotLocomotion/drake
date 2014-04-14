options.floating = true;
p = RigidBodyManipulator('Plane.URDF', options);
%p = RigidBodyManipulator('PlaneCollisionVisual.URDF', options);
%p = p.addRobotFromURDF('Plane.URDF',[],[],options);
p=p.addRobotFromURDF('Forest.xml');
%p=p.addRobotFromURDF('Forest2.xml');
v = p.constructVisualizer();
%    [x y z r p y Lw Rw  El Rd 
x0 = [-.5 0 0 0 0 0 .1 -.1 .4 -.1  0 0 0 0 0 0 0 0 0 0]';
%q = x0(1:10);
%kinsol = doKinematics(p,q);
%x0 = [x0;x0];
%x0(1) = 1;
v.draw(0,x0);
