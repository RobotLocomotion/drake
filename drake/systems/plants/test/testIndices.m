function testIndices

r = RigidBodyManipulator();

b = RigidBody();
b.linkname = 'Test';
b.jointname = 'TestJoint';
r = addLink(r,b);

b.linkname = 'Test2';
b.jointname = 'Test2Joint';
r = addLink(r,b);

valuecheck(findLinkId(r,'Test'),2);
valuecheck(findLinkId(r,'Test2'),3);

% make sure that the partial matches throw an error
try
  findLinkId(r,'Tes'); 
  error('should have thrown an error'); 
catch ex
  if ~strcmp(ex.identifier,'Drake:RigidBodyManipulator:UniqueLinkNotFound');
    rethrow(ex);
  end
end

valuecheck(findJointId(r,'TestJoint'),2);
valuecheck(findJointId(r,'Test2Joint'),3);


% make sure that the partial matches throw an error
try
  findJointId(r,'Test'); 
  error('should have thrown an error'); 
catch ex
  if ~strcmp(ex.identifier,'Drake:RigidBodyManipulator:UniqueJointNotFound');
    rethrow(ex);
  end
end
