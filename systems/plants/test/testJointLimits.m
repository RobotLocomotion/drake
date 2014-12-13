function testJointLimits()
r = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
[lb,ub] = r.getJointLimits;
ub_new = ub;
ub_new(13) = ub(13)+0.2;
lb_new = lb;
lb_new(14) = lb(14)+0.1;
r = r.setJointLimits(lb_new,ub_new);
r = r.compile();
[lb_new,ub_new] = r.getJointLimits();
valuecheck(lb(14)+0.1,lb_new(14));
valuecheck(ub(13)+0.2,ub_new(13));
end