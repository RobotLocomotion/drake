function ancestor_bodies = findAncestorBodies(obj, body_index)
%findAncestorBodies    Find indices of ancestors of a body
%   @param body_index index of a rigid body in this RigidBodyManipulator
%   @retval the indices of the parents of the rigid body with index
%   body_index, not including body_index, ordered from closest to most
%   distant ancestor

if (body_index==0)
  error('invalid body_index');
elseif (body_index < 0)
  fr = getFrame(obj,body_index);
  body_index = fr.body_ind;
end

ancestor_bodies = nan(getNumBodies(obj), 1); % conservative vector size
ancestor_index = 0;

body = obj.body(body_index);

while body.parent ~= 0 % body has a parent
  parent_index = body.parent;
  ancestor_index = ancestor_index + 1;
  ancestor_bodies(ancestor_index) = parent_index;
  
  body = obj.body(parent_index);
end

ancestor_bodies = ancestor_bodies(1 : ancestor_index);

end
