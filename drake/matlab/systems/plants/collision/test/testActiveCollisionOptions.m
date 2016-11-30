function testActiveCollisionOptions()
  urdf = [getDrakePath(), '/matlab/systems/plants/test/FallingBrick.urdf'];
  options.floating = true;
  options.terrain = RigidBodyFlatTerrain;

  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  r = RigidBodyManipulator(urdf,options);
  r = r.addGeometryToBody('world',RigidBodySphere(1),'groupA');
  r = compile(r);
  warning(w);

  x0 = Point(r.getStateFrame());
  x0.base_z = 2;

  kinsol = doKinematics(r,x0(1:r.getNumPositions()));

  if checkDependency('bullet')
    phi = r.collisionDetect(kinsol,false);
    valuecheck(phi,[1.5;0.5]);
  else
    phi = r.collisionDetect(kinsol,false);
    valuecheck(phi,[2.5;2.5;1.5;1.5;1.5;1.5;2.5;2.5]);
  end

  active_collision_options.terrain_only = true;
  phi = r.collisionDetect(kinsol,false,active_collision_options);
  valuecheck(phi,[2.5;2.5;1.5;1.5;1.5;1.5;2.5;2.5]);
  clear active_collision_options

  active_collision_options.collision_groups = {'default','terrain'};
  if checkDependency('bullet')
    phi = r.collisionDetect(kinsol,false,active_collision_options);
    valuecheck(phi,1.5);
  else
    phi = r.collisionDetect(kinsol,false,active_collision_options);
    valuecheck(phi,[2.5;2.5;1.5;1.5;1.5;1.5;2.5;2.5]);
  end

  active_collision_options.terrain_only = true;
  phi = r.collisionDetect(kinsol,false,active_collision_options);
  valuecheck(phi,[2.5;2.5;1.5;1.5;1.5;1.5;2.5;2.5]);
  clear active_collision_options

  active_collision_options.collision_groups = {'default','groupA'};
  if checkDependency('bullet')
    phi = r.collisionDetect(kinsol,false,active_collision_options);
    valuecheck(phi,0.5);
  else
    phi = r.collisionDetect(kinsol,false,active_collision_options);
    valuecheck(phi,[2.5;2.5;1.5;1.5;1.5;1.5;2.5;2.5]);
  end

  active_collision_options.terrain_only = true;
  phi = r.collisionDetect(kinsol,false,active_collision_options);
  valuecheck(phi,[2.5;2.5;1.5;1.5;1.5;1.5;2.5;2.5]);
  clear active_collision_options

  active_collision_options.collision_groups = {'groupA'};
  if checkDependency('bullet')
    phi = r.collisionDetect(kinsol,false,active_collision_options);
    sizecheck(phi,0);
  else
    phi = r.collisionDetect(kinsol,false,active_collision_options);
    sizecheck(phi,0);
  end

  active_collision_options.terrain_only = true;
  phi = r.collisionDetect(kinsol,false,active_collision_options);
  sizecheck(phi,0);
end
