function memoryTest

urdf = fullfile(getDrakePath,'examples','Acrobot','Acrobot.urdf');
r = RigidBodyManipulator(urdf);
getModel(r);
systems1 = find_system('SearchDepth',0);

r = RigidBodyManipulator(urdf);  % this causes the original rbm to be deleted.  verify if it also closes the simulink model
getModel(r);

systems2 = find_system('SearchDepth',0);

if numel(systems2)>numel(systems1)
  error('Simulink model was left open.  This will "leak" memory');
end

