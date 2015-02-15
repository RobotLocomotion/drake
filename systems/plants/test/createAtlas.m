%NOTEST
function robot = createAtlas(floatingJointType,options)
options.floating = floatingJointType;
if strcmp(floatingJointType,'quat')
  options.use_new_kinsol = true;
end
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
robot = RigidBodyManipulator(fullfile('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'),options);
warning(w);
end
