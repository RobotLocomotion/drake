function testContactConstraintsMex

checkDependency('bullet');

%build an atlas model 
robot = createAtlas('rpy');
if robot.mex_model_ptr == 0
  disp('testContactConstraintsMex: no mex model pointer... nothing to test');
  return;
end

%random initial pose
q = getRandomConfiguration(robot);

%get collision data
kinsol_no_gradients = robot.doKinematics(q);
[~,normal,xA,xB,idxA,idxB] = robot.collisionDetect(kinsol_no_gradients);
idxA = idxA';
idxB = idxB';

d = robot.surfaceTangents(normal);

%get the results from the mexed version
kinsol = robot.doKinematics(q, [], struct('compute_gradients', true));
[n_mex, D_mex] =  contactConstraintsmex(robot.mex_model_ptr, kinsol.mex_ptr, normal, int32(idxA - 1), int32(idxB - 1), xA, xB, d);
[n_mex, dn_mex] = eval(n_mex);

%get the results from the matlab version
[n, D, dn, dD] = robot.contactConstraintDerivatives(normal, kinsol, idxA, idxB, xA, xB, d);

%compare n
valuecheck(n_mex, n);
valuecheck(dn_mex, dn);

%compare D
for i = 1:size(D,2)
  [D_mexi, dD_mexi] = eval(D_mex{i});
  valuecheck(D_mexi, D{i});
  valuecheck(dD_mexi, dD{i});
end

end

