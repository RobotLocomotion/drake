function memoryTest

urdf = fullfile(getDrakePath,'examples','Acrobot','Acrobot.urdf');
r = RigidBodyManipulator(urdf);
getModel(r);
getModel(r);  % call it twice to make sure that's aok
systems1 = find_system('SearchDepth',0); vars1 = evalin('base','whos');

r = RigidBodyManipulator(urdf);  % this causes the original rbm to be deleted.  verify if it also closes the simulink model
getModel(r);

systems2 = find_system('SearchDepth',0); vars2 = evalin('base','whos');

if numel(systems2)>numel(systems1)
  error('Simulink model was left open.  This will "leak" memory');
end

if numel(vars2)>numel(vars1)
  error('adding something to the base workspace that didn''t get cleared up')
end
