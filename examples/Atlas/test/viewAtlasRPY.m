function viewAtlasRPY(rpy);

% just runs it as a passive system for now
options.floating = true;
r = TimeSteppingRigidBodyManipulator('urdf/atlas_minimal_contact.urdf',.001,options);
v = r.constructVisualizer;

x0 = double(Point(r.getStateFrame));

q=rpy2quat(rpy)
x0(1:6) = r.manip.floatingBaseFromQuat(zeros(3,1),q);
angles = x0(4:6)*(180/pi)
[xyz,quat] = r.manip.floatingBaseToQuat(x0(1:6))
valuecheck(q,quat);

%[x0(6),x0(5),x0(4)] = quat2angle(q','XYZ')
v.draw(0,double(x0));

