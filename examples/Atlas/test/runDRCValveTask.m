function runDRCValveTask
% doesn't actual demonstrate any functionality yet.  just a placeholder for now 

tmppath = addpathTemporary(fullfile(pwd,'..'));

options = struct();
r = Atlas('../urdf/atlas_minimal_contact.urdf',options);  % note: will need more contact for this
r = r.addRobotFromURDF('../urdf/simple_valve_wall.urdf',[1;0;0]);

load('../data/atlas_fp.mat', 'xstar');
x0 = r.resolveConstraints(xstar);
r = r.setInitialState(x0);

v = r.constructVisualizer();
v.drawWrapper(0,x0);
