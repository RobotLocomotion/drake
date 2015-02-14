function testContactConstraintsMex

checkDependency('bullet');

%build an atlas model 
p = RigidBodyManipulator(fullfile('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'));
if(p.mex_model_ptr == 0)
    disp('testContactConstraintsMex: no mex model pointer... nothing to test');
    return;
end

%random initial pose
nq = p.getNumPositions();
q0 = randn(nq, 1); 
kinsol = p.doKinematics(q0);

%get collision data
[~,normal,xA,xB,idxA,idxB] = p.collisionDetect(kinsol);
idxA = idxA';
idxB = idxB';
d = p.surfaceTangents(normal);

%get the results from the mexed version
[n_mex, D_mex, dn_mex, dD_mex] = contactConstraintDerivatives(p, kinsol, idxA, idxB, xA, xB, normal, d, true);

%get the results from the matlab version
[n, D, dn, dD] = contactConstraintDerivatives(p, kinsol, idxA, idxB, xA, xB, normal, d, false);

%compare n
assert(norm(n_mex - n) < 1e-6)

%compare D
for i = 1:size(D,2)
    assert(norm(D_mex{i} - D{i}) < 1e-6)
end

%compare dn
assert(norm(dn_mex - dn) < 1e-6)

%compare dD
for i = 1:size(dD,2)
    assert(norm(dD_mex{i} - dD{i}) < 1e-6)
end

end

