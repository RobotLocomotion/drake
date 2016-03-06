function testFrameParser()

compareParsers(fullfile(getDrakePath(), 'examples', 'Atlas', 'urdf', 'atlas_minimal_contact.urdf'));
compareParsers(fullfile(getDrakePath(), 'examples', 'PR2', 'pr2.urdf'));

end


function compareParsers(urdf)

robot = RigidBodyManipulator(urdf);

if robot.mex_model_ptr ~= 0
  testFrameParsermex(robot.mex_model_ptr, urdf);
end

end
