function radius = computeQ1LinFC(contact_pos,wrench_origin,friction_cone,Qw)
% compute the maximum radius of the ellipse, centered at the origin, and
% contained in the wrench polyhedron. The wrench polyhedron is defined as the convex
% hull of the vertices, mapped from the unit lengh friction cone edge.
% @param contact_pos  A 3 x num_contacts matrix. contact_pos(:,i) is the
% location of the i'th contact point
% %param wrench_orign A 3 x 1 vector, the point where the external wrench
% is applied.
% @param friction_cone  A num_contacts x 1 cell, friction_cone{i} is a 3 x
% num_edges_i matrix, where num_edges_i is the number of edges in the
% linearized friction cone, at the i'th point. friction_cone{i}(:,j) is the
% j'th edge at i'th contact point
% @param Qw    A 6 x 6 PSD matrix, the ellipse is defined as
% {w | w'*Qw*w<=radius^2}
% @retval radius  The radius of the largest L2 ball contained in the 
% wrench set. If force closure is not
% achieved, then this is -inf;
if(any(size(wrench_origin)~=[3,1]))
  error('wrench_origin should be a 3 x 1 vector');
end
num_pts = size(contact_pos,2);
G = graspTransferMatrix(contact_pos-bsxfun(@times,wrench_origin,ones(1,num_pts)));
if(numel(Qw) ~= 36 || size(Qw,1)~= 6 || size(Qw,2) ~= 6)
  error('Qw should be a 6 x 6 matrix');
end
if(min(eig(Qw)<0))
  error('Qw should be a positive-definite matrix');
end
if(numel(friction_cone) ~= num_pts)
  error('friction_cone should be a %d x 1 cell',num_pts);
end

% wrench_vertices(:,i) is a vertex of the wrench polyhedron, mapped from the
% unit length friction cone edge to the wrench space
wrench_vertices = [];
for i = 1:num_pts
  if(size(friction_cone{i},1) ~= 3)
    error('friction_cone{%d} should be a 3 x N matrix',i);
  end
  wrench_vertices = [wrench_vertices G(:,3*(i-1)+(1:3))*(friction_cone{i}./bsxfun(@times,sqrt(sum(friction_cone{i}.^2,1)),ones(3,1)))];
end

[wrench_polyhedron_A,wrench_polyhedron_b,wrench_polyhedron_Aeq,wrench_polyhedron_beq] = vert2lcon(wrench_vertices');
% The ellipse is contained in the wrench polyhedron, if w'*Qw*w<=
% rho_optimal implies wrench_polyhedron_A*w<=wrench_polyhedron_b
wrench_polyhedron_A = [wrench_polyhedron_A;wrench_polyhedron_Aeq;-wrench_polyhedron_Aeq];
wrench_polyhedron_b = [wrench_polyhedron_b;wrench_polyhedron_beq;-wrench_polyhedron_beq];
if(any(wrench_polyhedron_b<0))
  radius = -inf;
else
  radius = sqrt(min((wrench_polyhedron_b.^2)./sum((wrench_polyhedron_A.*(wrench_polyhedron_A/Qw)),2)));
end
end