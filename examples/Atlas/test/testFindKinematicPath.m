function testFindKinematicPath()

options.floating = true;
robot = RigidBodyManipulator(fullfile('../urdf/atlas_minimal_contact.urdf'),options);

num_bodies = robot.getNumBodies();

% verify that path to self contains only self
for i = 1 : num_bodies
  path = robot.findKinematicPath(i, i);
  valuecheck(path, i);
end


for i = 1 : num_bodies
  % verify that path to parent contains only self and parent and that path to
  % child is flipud(path from child to parent)
  parent = robot.getBody(i).parent;
  if parent ~= 0
    valuecheck(robot.findKinematicPath(i, parent), [i; parent]);
    valuecheck(robot.findKinematicPath(parent, i), [parent; i]);
  end

  for j = 1 : num_bodies
    path = robot.findKinematicPath(i, j);

    % verify that no paths have repeated entries
    valuecheck(length(unique(path)), length(path));

    % verify that all paths are connected
    assert(isLinkPathConnected(path, robot));

    % verify that all paths have at most one direction change
    assert(getDirectionChanges(path) < 2)

    if (getDirectionChanges(path) == 1)
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
