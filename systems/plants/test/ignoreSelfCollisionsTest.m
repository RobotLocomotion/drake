function ignoreSelfCollisionsTest()
  urdf_filename = fullfile(getDrakePath(),'examples','Atlas','urdf', ...
                           'atlas_convex_hull.urdf');
  options.ignore_self_collisions = true;
  options.floating = true;
  r = RigidBodyManipulator(urdf_filename,options);
  valuecheck(r.getNumContactPairs(),0);
end
