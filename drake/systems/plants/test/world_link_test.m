function world_link_test()

% Loads two URDFs, one with a world link and one without a world link.
% Verifies that the model created using these files correctly reports whether
% a world link is present in the model.

file_with_world = [getDrakePath '/systems/plants/test/models/'...
  'cylindrical_1dof_robot_fixed_to_world.urdf']
file_no_world = [getDrakePath '/systems/plants/test/models/'...
  'cylindrical_1dof_robot.urdf']

% Verifies an exception is thrown if a URDF is loaded with a world link defined.
test_passed = false
try
  model_with_world = RigidBodyManipulator(file_with_world, ...
    struct('floating','rpy'));
catch ex
  if strcmp(ex.identifier, 'Drake:WorldLinkInURDFModel')
    test_passed = true
  else
    rethrow(ex);
  end
end

if ~test_passed
  error(['ERROR: Did not catch expected exception caused by loading URDF', ...
    ' with world link]'])
end

% Verifies we can load a URDF that does not have a world link.
model_no_world = RigidBodyManipulator(file_no_world, struct('floating','rpy'));
