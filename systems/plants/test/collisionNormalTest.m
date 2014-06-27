options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
p = RigidBodyManipulator('shiftedPointMass.urdf',options);
q = zeros(6,1);
q(3) = .5;

[phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = p.contactConstraints(q,false,struct('terrain_only',false));
phi
normal

q(5) = 1e-6;

[phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = p.contactConstraints(q,false,struct('terrain_only',false));
phi
normal