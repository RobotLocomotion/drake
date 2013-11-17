function testPostureConstraint
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
r = RigidBodyManipulator(urdf,options);
nq = r.getNumDOF();

l_leg_kny = find(strcmp(r.getStateFrame.coordinates,'l_leg_kny'));
r_leg_kny = find(strcmp(r.getStateFrame.coordinates,'r_leg_kny'));

tspan0 = [0,1];
t = 0.5;
pc = PostureConstraint(r,tspan0);
pc = pc.setJointLimits([l_leg_kny;r_leg_kny],[0.2;0.2],inf(2,1));
[lb,ub] = pc.bounds(t);
[lbmex,ubmex] = testPostureConstraintmex(pc.mex_ptr,t);
valuecheck(lb,lbmex);
valuecheck(ub,ubmex);


pc = pc.setJointLimits([l_leg_kny;r_leg_kny],[-1;0.2],[1;inf]);
[lb,ub] = pc.bounds(t);
[lbmex,ubmex] = testPostureConstraintmex(pc.mex_ptr,t);
valuecheck(lb,lbmex);
valuecheck(ub,ubmex);
end
