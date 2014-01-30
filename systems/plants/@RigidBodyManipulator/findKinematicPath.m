function [body_path, joint_path, signs] = findKinematicPath(obj, start_body, end_body)
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



ancestors1 = [start_body; findAncestorBodies(obj, start_body)];
ancestors2 = [end_body; findAncestorBodies(obj, end_body)];

commonSize = min(size(ancestors1, 1), size(ancestors2, 1));

reducedAncestors1 = ancestors1(end - commonSize + 1: end);
reducedAncestors2 = ancestors2(end - commonSize + 1: end);

commonIndices = reducedAncestors1 == reducedAncestors2;
lca_index = find(commonIndices, 1);
if isempty(lca_index)
  error(['there is no path between ' start_body ' and ' end_body]);
end

least_common_ancestor = reducedAncestors1(lca_index);

path_start = ancestors1(1 : size(ancestors1, 1) - commonSize + lca_index - 1);
path_end = flipud(ancestors2(1 : size(ancestors2, 1) - commonSize + lca_index - 1));

body_path = [path_start; least_common_ancestor; path_end];
joint_path = [path_start; path_end];
signs = [-ones(size(path_start)); ones(size(path_end))];


% THIS IS MUCH SLOWER:
% start_body_ancestors = [start_body; obj.findAncestorBodies(start_body)];
% end_body_ancestors = [end_body; obj.findAncestorBodies(end_body)];
% [common_ancestors, start_indices, end_indices] = intersect(start_body_ancestors, end_body_ancestors);
% if isempty(common_ancestors)
%   error(['there is no path between ' start_body ' and ' end_body]);
% end
% 
% [least_common_ancestor, least_common_ancestor_index] = max(common_ancestors);
% 
% start_index = start_indices(least_common_ancestor_index);
% path_start = start_body_ancestors(1 : start_index - 1);
% 
% end_index = end_indices(least_common_ancestor_index);
% path_end = flipud(end_body_ancestors(1 : end_index - 1));
% 
% body_path = [path_start; least_common_ancestor; path_end];
% joint_path = [path_start; path_end];
% signs = [-ones(size(path_start)); ones(size(path_end))];

end

