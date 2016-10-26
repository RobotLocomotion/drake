function ignoreSelfCollisionsTest()
  urdf_filename = fullfile(getDrakePath(),'examples','Atlas','urdf', ...
                           'atlas_convex_hull.urdf');
  options.ignore_self_collisions = true;
  options.floating = true;
  r = RigidBodyManipulator(urdf_filename,options);
  valuecheck(r.getNumContactPairs(),0);

  if (checkDependency('bullet'))
    options.ignore_self_collisions = false;
    options.floating = true;
    r = RigidBodyManipulator(urdf_filename,options);
    if r.getNumContactPairs()==0,
      error('should have contact pairs');
    end
  end
end
