options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
p = RigidBodyManipulator('../KneedCompassGait.urdf',options);
q = zeros(p.getNumPositions,1);
q(3) = 1;

q2 = q;
q2(4) = 1e-6;
[phi,normal,d,xA,xB,idxA,idxB] = p.contactConstraints(q,false,struct('terrain_only',true));
[phi2,normal2,d2,xA2,xB2,idxA2,idxB2] = p.contactConstraints(q2,false,struct('terrain_only',true));
