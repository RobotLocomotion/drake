function singleJointParameterEstimation

r = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf');
shoulder = findJointId(r,'l_arm_shz');

for i=1:getNumBodies(r)
  if i~=shoulder
    r = weldJoint(r,i);
  end
end

r = compile(r);
