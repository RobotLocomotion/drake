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
kinsol = robot.doKinematics(q);

%get collision data
[~,normal,xA,xB,idxA,idxB] = robot.collisionDetect(kinsol);
idxA = idxA';
idxB = idxB';

%get the results from the mexed version
[d_mex, n_mex, D_mex, dn_mex, dD_mex] =  contactConstraintsmex(robot.mex_model_ptr, normal, int32(idxA), int32(idxB), xA, xB);

%get the results from the matlab version
[d, n, D, dn, dD] = robot.contactConstraintDerivatives(normal, kinsol, idxA, idxB, xA, xB);

%compare d
for i = 1:size(d,2)
  valuecheck(d_mex{i}, d{i});
end

%compare n
valuecheck(n_mex, n);

%compare D
for i = 1:size(D,2)
  valuecheck(D_mex{i}, D{i});
end

%compare dn
valuecheck(dn_mex, dn);

%compare dD
for i = 1:size(dD,2)
  valuecheck(dD_mex{i}, dD{i});
end

end

