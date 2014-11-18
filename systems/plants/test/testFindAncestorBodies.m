function testFindAncestorBodies()

robot = createAtlas('rpy');

num_bodies = robot.getNumBodies();

for i = 1 : num_bodies
  ancestor_bodies = robot.findAncestorBodies(i);
  
  % verify that root bodies have no ancestors
  if robot.getBody(i).parent == 0
    valuecheck(length(ancestor_bodies), 0);
  end
  
  % verify that all ancestors have a lower index than the current body
  for j = 1 : length(ancestor_bodies)
    assert(j < i)
  end
  
  % verify that ancestors of body i are ancestors of parent of body + the
  % parent of body i
  parent = robot.getBody(i).parent;
  if (parent ~= 0)
    parent_ancestor_bodies = robot.findAncestorBodies(parent);
    valuecheck(ancestor_bodies, [parent; parent_ancestor_bodies]);
  end
  
end

end

