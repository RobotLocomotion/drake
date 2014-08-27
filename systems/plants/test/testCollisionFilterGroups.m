function testCollisionFilterGroups()
  urdf = fullfile(getDrakePath(),'examples','KneedCompassGait','KneedCompassGait.urdf');
  options.floating = false; % this forces the link 'hip' to be welded to the
                            % world
  options.terrain = RigidBodyFlatTerrain();
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  r = RigidBodyManipulator(urdf,options);
  warning(w);
end
