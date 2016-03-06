function normals = mesh_normals(vertices, faces)
% Compute the normals to a 3D mesh. This assumes that the elements of 
% the faces matrix are ordered to cycle clockwise when observed from 
% outside the body. 

normals = zeros(3, size(faces,2));
for j = 1:size(faces, 2)
  normals(:,j) = -cross(vertices(:,faces(2,j)) - vertices(:,faces(1,j)),...
                       vertices(:,faces(3,j)) - vertices(:,faces(2,j)));
  normals(:,j) = normals(:,j) / norm(normals(:,j));
end