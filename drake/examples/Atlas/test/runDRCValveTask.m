function runDRCValveTask
% doesn't actual demonstrate any functionality yet.  just a placeholder for now 

tmppath = addpathTemporary(fullfile(pwd,'..'));

options.floating = true;
options.replace_cylinders_with_capsules = false;
r = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);  % note: will need more contact for this
r = r.addRobotFromURDF('../urdf/valve_wall.urdf',[1;0;0],[0;0;pi],options);

load('../data/atlas_fp.mat', 'xstar');
x0 = mergeCoordinates(getStateFrame(r),{xstar,zeros(getNumStates(r)-numel(xstar),1)});
x0 = r.resolveConstraints(x0);

%options.use_collision_geometry = true;
v = r.constructVisualizer(options);
%v.drawWrapper(0,x0);
v.inspector(x0)