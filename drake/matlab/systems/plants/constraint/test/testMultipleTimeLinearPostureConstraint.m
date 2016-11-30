function testMultipleTimeLinearPostureConstraint

checkDependency('rigidbodyconstraint_mex');

urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();

l_leg_kny = find(strcmp(r.getStateFrame.getCoordinateNames(),'l_leg_kny'));
r_leg_kny = find(strcmp(r.getStateFrame.getCoordinateNames(),'r_leg_kny'));

tspan0 = [0,1];
t = [-1 0 0.2 0.4 0.7 1 2];
q = zeros(nq,length(t));
for i = 1 : length(t)
  q(:, i) = getRandomConfiguration(r);
end

display('Check posture change constraint');
testMTLPC_userfun(t,q,RigidBodyConstraint.PostureChangeConstraintType,r,[l_leg_kny;r_leg_kny],[-0.1;-0.2],[0.2;0],tspan0);

end

function testMTLPC_userfun(t,q,mtlpc_type,varargin)
robot = varargin{1};
robot_ptr = robot.getMexModelPtr;
mtlpc_mex = constructRigidBodyConstraint(mtlpc_type,true,robot_ptr,varargin{2:end});
mtlpc = constructRigidBodyConstraint(mtlpc_type,false,robot,varargin{2:end});
category_name_mex = constraintCategorymex(mtlpc.mex_ptr);
if(~strcmp(category_name_mex,mtlpc.categoryString()))
  error('category name string do not match')
end
[num_cnst_mex,c_mex,iAfun_mex,jAvar_mex,A_mex,cnst_name_mex,lb_mex,ub_mex] = testMultipleTimeLinearPostureConstraintmex(mtlpc_mex,q,t);
valuecheck(c_mex,sparse(iAfun_mex,jAvar_mex,A_mex,num_cnst_mex,numel(q))*q(:),1e-10);
num_cnst = mtlpc.getNumConstraint(t);
c = mtlpc.feval(t,q);
[iAfun,jAvar,A] = mtlpc.geval(t);
cnst_name = mtlpc.name(t);
[lb,ub] = mtlpc.bounds(t);
valuecheck(num_cnst_mex,num_cnst);
valuecheck(c_mex,c,1e-10);
valuecheck(iAfun_mex,iAfun);
valuecheck(jAvar_mex,jAvar);
valuecheck(A_mex,A,1e-10);
valuecheck(lb_mex,lb,1e-10);
valuecheck(ub_mex,ub,1e-10);
strcmp(cnst_name_mex,cnst_name);
cnstr = mtlpc.generateConstraint(t);
valuecheck(strcmp(cnst_name,cnstr{1}.name),1);
sizecheck(cnstr,[1,1]);
valuecheck(cnstr{1}.lb,lb);
valuecheck(cnstr{1}.ub,ub);
A_mat = sparse(iAfun,jAvar,A,num_cnst,mtlpc.robot.getNumPositions*length(t));
valuecheck(cnstr{1}.A,A_mat);
end
