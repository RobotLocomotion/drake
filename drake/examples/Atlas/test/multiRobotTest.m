function multiRobotTest

urdf = '../urdf/atlas_minimal_contact.urdf';
options.view = 'right';
r = PlanarRigidBodyManipulator('',options);

w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = addRobotFromURDF(r,urdf,[-1;2]);
r = addRobotFromURDF(r,urdf,[1;2]);
warning(w);

% add a quick test of the multi frames
% which finds a transform from the full state
% to the position vector of the first atlas
options.throw_error_if_fail = true;
tf = findTransform(getStateFrame(r),getPositionFrame(r,2),options);
valuecheck(tf.output(0,[],(1:getNumStates(r))'),(15:28)');

% now simulate and playback
xtraj = simulate(r,[0 3]);

v = r.constructVisualizer();
v.playback(xtraj);

