function testPostureConstraint

checkDependency('rigidbodyconstraint_mex');

urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();

l_leg_kny = find(strcmp(r.getStateFrame.getCoordinateNames(),'l_leg_kny'));
r_leg_kny = find(strcmp(r.getStateFrame.getCoordinateNames(),'r_leg_kny'));

tspan0 = [0,1];
t = 0.5;
pc = PostureConstraint(r,tspan0);
pc = pc.setJointLimits([l_leg_kny;r_leg_kny],[0.2;0.2],inf(2,1));
category_name_mex = constraintCategorymex(pc.mex_ptr);
if(~strcmp(category_name_mex,pc.categoryString()))
  error('category name string do not match')
end
[lb,ub] = pc.bounds(t);
[lbmex,ubmex] = testPostureConstraintmex(pc.mex_ptr,t);
valuecheck(lb,lbmex);
valuecheck(ub,ubmex);

display('Check setJointLimits when input is outside of the robot default joint limit');
pc = pc.setJointLimits([l_leg_kny;r_leg_kny],[-1;0.2],[1;inf]);
[lb,ub] = pc.bounds(t);
[lbmex,ubmex] = testPostureConstraintmex(pc.mex_ptr,t);
valuecheck(lb,lbmex);
valuecheck(ub,ubmex);

display('Check if the mex pointer and MATLAB object are consistent after copy by value');
pc2 = pc;
[lb2,ub2] = pc2.bounds(t);
[lb2_mex,ub2_mex] = testPostureConstraintmex(pc2.mex_ptr,t);
valuecheck(lb2,lb2_mex);
valuecheck(ub2,ub2_mex);
pc2 = pc2.setJointLimits(1,0,0);
[lb2,ub2] = pc2.bounds(t);
valuecheck(lb2(1),0);
valuecheck(ub2(1),0);
[lb2_mex,ub2_mex] = testPostureConstraintmex(pc2.mex_ptr,t);
valuecheck(lb2_mex(1),0);
valuecheck(ub2_mex(1),0);
[lb,ub] = pc.bounds(t);
valuecheck(lb(1),-inf);
valuecheck(ub(1),inf);
[lb_mex,ub_mex] = testPostureConstraintmex(pc.mex_ptr,t);
valuecheck(lb_mex(1),-inf);
valuecheck(ub_mex(1),inf);

display('Check generateConstraint function')
pc_bbcnstr = pc2.generateConstraint(t);
valuecheck(pc_bbcnstr{1}.lb,lb2);
valuecheck(pc_bbcnstr{1}.ub,ub2);
end
