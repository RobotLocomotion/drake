function [body_path, joint_path, signs] = findKinematicPath(obj, start_body, end_body, use_mex)
% findKinematicPath computes the shortest path from one robot link to
% another
%
% @param start_body index of rigid body at which the path should start
% @param end_body index of rigid body at which the path should end
% @return body_path column vector of rigid body indices that represent the
% shortest path starting at start_body and ending at end_body, including
% start_body and end_body
% @return joint_path indices of joints on shortest path from start_body to
% end_body
% @return signs vector of traversal directions along joint path. +1 if
% joint is traversed from child to parent, -1 if joint is traversed from
% parent to child
% Throws an error if there is no path between the bodies
%

if nargin < 4
  use_mex = obj.mex_model_ptr ~= 0;
end

if use_mex
  [body_path, joint_path, signs] = findKinematicPathmex(obj.mex_model_ptr, start_body - 1, end_body - 1); % base 1 to base 0
  % base 0 to base 1:
  body_path = body_path + 1;
  joint_path = joint_path + 1;
else
  if (start_body < 0)
    fr = getFrame(obj,start_body);
    start_body = fr.body_ind;
  end
  
  if (end_body < 0)
    fr = getFrame(obj,end_body);
    end_body = fr.body_ind;
  end
  
  ancestors1 = [start_body; findAncestorBodies(obj, start_body)];
  ancestors2 = [end_body; findAncestorBodies(obj, end_body)];
  
  common_size = min(size(ancestors1, 1), size(ancestors2, 1));
  
  reduced_ancestors1 = ancestors1(end - common_size + 1: end);
  reduced_ancestors2 = ancestors2(end - common_size + 1: end);
  
  common_indices = reduced_ancestors1 == reduced_ancestors2;
  lca_index = find(common_indices, 1);
  if isempty(lca_index)
    error(['there is no path between ' start_body ' and ' end_body]);
  end
  
  least_common_ancestor = reduced_ancestors1(lca_index);
  
  path_start = ancestors1(1 : size(ancestors1, 1) - common_size + lca_index - 1);
  path_end = flipud(ancestors2(1 : size(ancestors2, 1) - common_size + lca_index - 1));
  
  body_path = [path_start; least_common_ancestor; path_end];
  if isempty(path_start) && isempty(path_end)
    joint_path = zeros(0, 1);
    signs = zeros(0, 1);
    % otherwise it would be 0 x 2, which doesn't make sense
  else
    joint_path = [path_start; path_end];
    signs = [-ones(size(path_start)); ones(size(path_end))];
  end
end

