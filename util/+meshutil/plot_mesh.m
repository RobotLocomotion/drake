function plot_mesh(vertices, faces, options)

if nargin < 3
  options = struct();
end
if ~isfield(options, 'normals'); options.normals = []; end
if ~isfield(options, 'patch_spec'); options.patch_spec = {'k', 'FaceAlpha', 0.5}; end

hold on
for j = 1:size(faces, 2)
  pts = vertices(:,faces(:,j)');
  patch(pts(1,:), pts(2,:), pts(3,:), options.patch_spec{:});
%   patch(pts(1,:), pts(2,:), pts(3,:), [.7,.7,.7], 'FaceAlpha', 1, 'EdgeColor','k');
end


for j = 1:size(options.normals, 2)
  center = mean(vertices(:,faces(:,j)'), 2);
  quiver3(center(1), center(2), center(3), normals(1,j), options.normals(2,j), options.normals(3,j))
end
