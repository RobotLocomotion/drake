function addRobotFromURDFStringTest()
  % Checks that RBMs generated from files and from strings match.
  files = { ...
    fullfile(getDrakePath(), 'examples', 'Acrobot', ...
             'Acrobot.urdf'), ...
    fullfile(getDrakePath(), 'examples', 'UnderwaterAcrobot', ...
             'UnderwaterAcrobot.urdf'), ...
    fullfile(getDrakePath(), 'examples', 'KneedCompassGait', ...
             'KneedCompassGait.urdf') ...
  };
  for f = files
    urdf_filename = f{1};
    testGeneratedManipulators(urdf_filename);
  end
end

function testGeneratedManipulators(urdf_filename)
  urdf_string = fileread(urdf_filename);
  r_file = RigidBodyManipulator();
  r_file = r_file.addRobotFromURDF(urdf_filename);
  r_string = RigidBodyManipulator();
  r_string = r_string.addRobotFromURDFString(urdf_string);

  % Check that the models are the same modulo those properties which are
  % always unique
  r_file_struct = struct(r_file);
  r_string_struct = struct(r_string);
  fields = fieldnames(r_file_struct);
  fields = setdiff(fields, {'mex_model_ptr', 'uid', 'urdf'});
  for f = reshape(fields, 1, [])
    field = f{1};
    assert(isfield(r_string_struct, field));
    assert(isequal(r_string_struct.(field), r_file_struct.(field)));
  end
end
