function fc_edges = linFCedges(num_fc_edges,mu_face)
% Given the number of edges in the linearized friction cone and friction
% coefficient, return the edges of the cone, supposing the normal vector of
% the cone is [0;0;1]
% @param num_fc_edges   Number of edges in the linearized friction cone
% @param mu_face    The friction coefficient of the linearized friction
% cone
% @retval fc_edges   A 3 x num_fc_edges matrix, each column of fc_edges is a
% unit length vector
if(numel(num_fc_edges) ~= 1 || num_fc_edges<1)
  error('num_fc_edges should be a positive integer');
end
if(numel(mu_face) ~= 1 || mu_face<0)
  error('mu_face should be a non-negative scalar');
end
fc_theta = linspace(0,2*pi,num_fc_edges+1);
fc_theta = fc_theta(1:end-1);
fc_edges = [mu_face*cos(fc_theta);mu_face*sin(fc_theta);ones(1,num_fc_edges)];
fc_edges = fc_edges./bsxfun(@times,ones(3,1),sqrt(sum(fc_edges.^2,1)));
end