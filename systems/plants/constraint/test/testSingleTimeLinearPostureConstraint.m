function testSingleTimeLinearPostureConstraint
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
robot = RigidBodyManipulator(urdf,options);
coords = robot.getStateFrame.coordinates(1:robot.getNumDOF);
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
q = randn(robot.getNumDOF,1);
stlpc = SingleTimeLinearPostureConstraint(robot,iAfun,jAvar,A,lb,ub,tspan);
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
end

function testSingleTimeLinearPostureConstraint_userfun(stlpc,q,t)
stlpc_ptr = stlpc.mex_ptr;
num_cnst = stlpc.getNumConstraint(t);
c = stlpc.feval(t,q);
[iAfun,jAvar,A] = stlpc.geval(t);
cnst_name = stlpc.name(t);
[lb,ub] = stlpc.bounds(t);
[num_cnst_mex,c_mex,iAfun_mex,jAvar_mex,A_mex,cnst_name_mex,lb_mex,ub_mex] = testSingleTimeLinearPostureConstraintmex(stlpc_ptr,q,t);
valuecheck(num_cnst,num_cnst_mex);
valuecheck(c,c_mex,1e-10);
valuecheck(iAfun,iAfun_mex);
valuecheck(jAvar,jAvar_mex);
valuecheck(A,A_mex,1e-10);
valuecheck(lb,lb_mex);
valuecheck(ub,ub_mex);
end