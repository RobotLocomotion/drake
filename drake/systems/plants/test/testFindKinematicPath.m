function testFindKinematicPath()

robot = createAtlas('rpy');

testPathToSelf(robot);
testAllPaths(robot);

end

function testPathToSelf(robot)
% verify that path to self contains only self
num_bodies = robot.getNumBodies();
for i = 1 : num_bodies
  path = robot.findKinematicPath(i, i, false);
  valuecheck(path, i);
end
end

function testAllPaths(robot)

compare_to_mex = exist('findKinematicPathmex','file') == 3;

num_bodies = robot.getNumBodies();
for i = 1 : num_bodies
  % verify that path to parent contains only self and parent and that path to
  % child is flipud(path from child to parent)
  parent = robot.getBody(i).parent;
  if parent ~= 0
    valuecheck(robot.findKinematicPath(i, parent, false), [i; parent]);
    valuecheck(robot.findKinematicPath(parent, i, false), [parent; i]);
  end
  
  for j = 1 : num_bodies
    [body_path, joint_path, signs] = robot.findKinematicPath(i, j, false);

    if compare_to_mex
      [body_path_mex, joint_path_mex, signs_mex] = robot.findKinematicPath(i, j, true);
      valuecheck(body_path_mex, body_path);
      valuecheck(joint_path_mex, joint_path);
      valuecheck(signs_mex, signs);
    end
    
    % verify that no paths have repeated entries
    valuecheck(length(unique(body_path)), length(body_path));
    
    % verify that all paths are connected
    assert(isLinkPathConnected(body_path, robot));
    
    % verify that all paths have at most one direction change
    assert(getDirectionChanges(body_path) < 2)
    
    if (getDirectionChanges(body_path) == 1)
    end
    
    %     if (getDirectionChanges(path) > 1)
    %       printBodyNames(path, robot);
    %       robot.drawKinematicTree();
    %     end
  end
end
end

function ret = isLinkPathConnected(path, robot)
ret = true;
path_length = length(path);
for i = 1 : path_length
  j = i + 1;
  if j <= path_length
    a = path(i);
    b = path(j);
    
    is_a_parent_of_b = robot.getBody(a).parent == b;
    is_b_parent_of_a = robot.getBody(b).parent == a;
    if ~(is_a_parent_of_b || is_b_parent_of_a)
      ret = false;
    end
  end
end
end

function ret = getDirectionChanges(path)
changes = diff(path);

signs_of_changes = sign(changes);

ret = 0;
n = length(signs_of_changes);
for i = 1 : n
  j = i + 1;
  if j <= n
    if signs_of_changes(i) ~= signs_of_changes(j)
      ret = ret + 1;
    end
  end
end

end

function printBodyNames(path, robot)
for i = 1 : length(path)
  disp(robot.getLinkName(path(i)))
end
end
