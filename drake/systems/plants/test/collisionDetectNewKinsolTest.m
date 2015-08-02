function collisionDetectNewKinsolTest()
  urdf = fullfile(getDrakePath(), 'examples', 'Atlas', 'urdf', 'atlas_convex_hull.urdf');
  options.floating = true;
  options.use_new_kinsol = true;
  options.terrain = RigidBodyFlatTerrain();
  w = warning('off', 'Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off', 'Drake:RigidBodyManipulator:UnsupportedContactPoints');
  r_new = RigidBodyManipulator(urdf, options);
  options.use_new_kinsol = false;
  r = RigidBodyManipulator(urdf, options);
  warning(w);
  S = load(fullfile(getDrakePath(), 'examples', 'Atlas', 'data', 'atlas_fp.mat'));
  q = S.xstar(1:r.getNumPositions());
  q(3) = 10;

  phi = r.collisionDetect(q);
  phi_new = r_new.collisionDetect(q);

  valuecheck(phi_new, phi, 1e-12);
end
