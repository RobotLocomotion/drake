function plot_mesh(vertices, faces, normals)

hold on
for j = 1:size(faces, 2)
  pts = vertices(:,faces(:,j)');
  patch(pts(1,:), pts(2,:), pts(3,:), 'k', 'FaceAlpha', 0.5);
end

if nargin > 2
  for j = 1:size(faces, 2)
    center = mean(vertices(:,faces(:,j)'), 2);
    quiver3(center(1), center(2), center(3), normals(1,j), normals(2,j), normals(3,j))
  end
end
