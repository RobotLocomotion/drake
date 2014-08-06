function testActiveCollisionOptions()
  urdf = [getDrakePath(), '/systems/plants/test/FallingBrick.urdf'];
  options.floating = true;
  options.terrain = RigidBodyFlatTerrain;

  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  r = RigidBodyManipulator(urdf,options);
  r = r.addShapeToBody('world',RigidBodySphere(1),'groupA');
  r = compile(r);
  warning(w);

  x0 = Point(r.getStateFrame());
  x0.base_z = 2;

  kinsol = doKinematics(r,x0(1:r.getNumPositions()));

  phi = r.collisionDetect(kinsol,false);
  valuecheck(phi,[1.5;0.5]);

  active_collision_options.collision_groups = {'default','terrain'};
  phi = r.collisionDetect(kinsol,false,active_collision_options);
  valuecheck(phi,1.5);

  active_collision_options.collision_groups = {'default','groupA'};
  phi = r.collisionDetect(kinsol,false,active_collision_options);
  valuecheck(phi,0.5);

  active_collision_options.collision_groups = {'terrain','groupA'};
  phi = r.collisionDetect(kinsol,false,active_collision_options);
  sizecheck(phi,0);
end
