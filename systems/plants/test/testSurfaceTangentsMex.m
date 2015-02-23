function testSurfaceTangentsMex

checkDependency('bullet');

%build an atlas model 
p = RigidBodyManipulator(fullfile('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'));
if(p.mex_model_ptr == 0)
    disp('testSurfaceTangentsMex: no mex model pointer... nothing to test');
    return;
end

%random initial pose
nq = p.getNumPositions();
q0 = randn(nq, 1); 
kinsol = p.doKinematics(q0);

%get collision data
[~,normal,~,~,~,~] = p.collisionDetect(kinsol);

%get the results from the mexed version
d_mex  = contactConstraintDerivatives(p, true, normal);

%get the results from the matlab version
d = contactConstraintDerivatives(p, false, normal);

%compare d
for i = 1:size(d,2)
    assert(norm(d_mex{i} - d{i}) < 1e-6)
end

end

