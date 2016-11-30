function [Jdot_times_v, dJdot_times_v] = terrainContactJacobianDotTimesV(obj, ...
  kinsol, varargin)
% Computes dJ/dt * v, where J is the matrix that maps the velocity vector
% v to a stacked vector of terrain contact point velocities.

compute_gradient = nargout > 1;

if numel(varargin) > 0 &&  isstruct(varargin{1})
  terrain_contact_point_struct = varargin{1};
else
  terrain_contact_point_struct = getTerrainContactPoints(obj,varargin{:});
end

n_pts = arrayfun(@(x) size(x.pts,2),terrain_contact_point_struct);
cumsum_n_pts = cumsum([0,n_pts]);

Jdot_times_v = zeros(3*cumsum_n_pts(end), 1) * kinsol.q(1);
if compute_gradient
  nq = obj.getNumPositions();
  dJdot_times_v = zeros(numel(Jdot_times_v), nq);
end

for i = 1:numel(terrain_contact_point_struct)
  body_or_frame_index = terrain_contact_point_struct(i).idx;
  points = terrain_contact_point_struct(i).pts;
  indices = 3*cumsum_n_pts(i)+(1:3*n_pts(i));
  if compute_gradient
    [Jdot_times_v(indices), dJdot_times_v(indices, :)] = forwardJacDotTimesV(obj, kinsol, body_or_frame_index, points, 0, 1);
  else
    Jdot_times_v(indices) = forwardJacDotTimesV(obj, kinsol, body_or_frame_index, points, 0, 1);
  end
end

end