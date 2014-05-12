function normals = mesh_normals(vertices, faces)

normals = zeros(3, size(faces,2));
for j = 1:size(faces, 2)
  normals(:,j) = -cross(vertices(:,faces(2,j)) - vertices(:,faces(1,j)),...
                       vertices(:,faces(3,j)) - vertices(:,faces(2,j)));
  normals(:,j) = normals(:,j) / norm(normals(:,j));
end