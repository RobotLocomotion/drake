function contactNormalsTest()
  options.terrain = RigidBodyFlatTerrain();
  options.floating = true;
  options.ignore_self_collisions = true;
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  p = RigidBodyManipulator('ShiftedPointMass.urdf',options);
  warning(w);
  q = zeros(6,1);
  q(3) = .51;

  [phi,normal,~,xA] = p.contactConstraints(q,false,struct('terrain_only',false));
  %phi
  %normal
  %xA

  q(5) = 1e-6;

  [phi2,normal2,~,xA2] = p.contactConstraints(q,false,struct('terrain_only',false));
  %phi2
  %normal2
  %xA2

  valuecheck(phi+(1-cos(1e-6))*0.5,phi2);
  valuecheck(normal,normal2);
  valuecheck(xA-[0.5*sin(1e-6);0;0],xA2,1e-6);
end
