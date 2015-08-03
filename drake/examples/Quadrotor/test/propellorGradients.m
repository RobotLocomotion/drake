function propellorGradients

p = RigidBodyManipulator('../quadrotor.urdf',struct('floating',true));
prop = p.force{1};

for k = 1:20
    q = rand(p.getNumPositions,1);
    v = rand(p.getNumVelocities,1);

    [~,~,~,dB_mod_numerical] = geval(2,@(q,v)prop.computeSpatialForce(p,q,v),q,v,struct('grad_method','numerical'));
    [~,~,~,dB_mod_user] = prop.computeSpatialForce(p,q,v);
    
    valuecheck(dB_mod_numerical,dB_mod_user,1e-5);
    
end

end