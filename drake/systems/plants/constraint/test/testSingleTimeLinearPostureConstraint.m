function testSingleTimeLinearPostureConstraint
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
robot = RigidBodyManipulator(urdf,options);
coords = robot.getStateFrame.getCoordinateNames();
coords = coords(1:robot.getNumPositions);
l_leg_kny = find(strcmp(coords,'l_leg_kny'));
r_leg_kny = find(strcmp(coords,'r_leg_kny'));
l_leg_hpy = find(strcmp(coords,'l_leg_hpy'));
r_leg_hpy = find(strcmp(coords,'r_leg_hpy'));
l_leg_aky = find(strcmp(coords,'l_leg_aky'));
r_leg_aky = find(strcmp(coords,'r_leg_aky'));
l_leg_hpz = find(strcmp(coords,'l_leg_hpz'));
r_leg_hpz = find(strcmp(coords,'r_leg_hpz'));
iAfun = [1;1;2;2;3;3;4;4];
jAvar = [l_leg_kny;r_leg_kny;l_leg_hpy;r_leg_hpy;l_leg_aky;r_leg_aky;l_leg_hpz;r_leg_hpz];
A = [1;-1;1;-1;1;-1;1;-1];
lb = [0;0;0;-0.1*pi];
ub = [0;0;0;0.1*pi];
tspan = [0 1];
q = getRandomConfiguration(robot);
stlpc = SingleTimeLinearPostureConstraint(robot,iAfun,jAvar,A,lb,ub,tspan);

% todo: move this farther down, so that more tests are included when this
% dependency is not satisfied...
checkDependency('rigidbodyconstraint_mex')

category_name_mex = constraintCategorymex(stlpc.mex_ptr);
if(~strcmp(category_name_mex,stlpc.categoryString()))
  error('category name string do not match')
end
display('Check t within tspan');
t = 0;
num_cnst = stlpc.getNumConstraint(t);
valuecheck(num_cnst,4);
testSingleTimeLinearPostureConstraint_userfun(stlpc,q,t);
display('Check t outside of tspan')
t = -1;
num_cnst = stlpc.getNumConstraint(t);
valuecheck(num_cnst,0);
testSingleTimeLinearPostureConstraint_userfun(stlpc,q,t);
display('Check empty t')
t = [];
num_cnst = stlpc.getNumConstraint(t);
valuecheck(num_cnst,4);
testSingleTimeLinearPostureConstraint_userfun(stlpc,q,t);
display('Check constraint value');
q(l_leg_kny) = q(r_leg_kny);
q(l_leg_hpy) = q(r_leg_hpy);
q(l_leg_aky) = q(r_leg_aky);
q(l_leg_hpz) = q(r_leg_hpz)+0.05*pi;
c_val = stlpc.feval(t,q);
if(any(c_val>ub+1e-10) || any(c_val<lb-1e-10))
  error('constraint value is incorrect');
end

cnstr = stlpc.generateConstraint(t);
sizecheck(cnstr,[1,1]);
valuecheck(cnstr{1}.lb,stlpc.lb);
valuecheck(cnstr{1}.ub,stlpc.ub);
valuecheck(cnstr{1}.A,stlpc.A_mat);
end

function testSingleTimeLinearPostureConstraint_userfun(stlpc,q,t)
stlpc_ptr = stlpc.mex_ptr;
num_cnst = stlpc.getNumConstraint(t);
c = stlpc.feval(t,q);
[iAfun,jAvar,A] = stlpc.geval(t);
cnst_name = stlpc.name(t);
[lb,ub] = stlpc.bounds(t);
[type,num_cnst_mex,c_mex,iAfun_mex,jAvar_mex,A_mex,cnst_name_mex,lb_mex,ub_mex] = testSingleTimeLinearPostureConstraintmex(stlpc_ptr,q,t);
valuecheck(type,stlpc.type);
if(~strcmp(constraintTypemex(stlpc.mex_ptr),stlpc.mex_ptr.name))
  error('constraint type do not match');
end
valuecheck(num_cnst,num_cnst_mex);
valuecheck(c,c_mex,1e-10);
valuecheck(iAfun,iAfun_mex);
valuecheck(jAvar,jAvar_mex);
valuecheck(A,A_mex,1e-10);
valuecheck(lb,lb_mex);
valuecheck(ub,ub_mex);
end