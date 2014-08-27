function testCollisionFilterGroups()
  %urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
  urdf = fullfile(getDrakePath(),'examples','KneedCompassGait','KneedCompassGait.urdf');
  options.floating = false; % this forces the link 'hip' to be welded to the
                            % world
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  r = RigidBodyManipulator(urdf,options);
  warning(w);
  keyboard
end
