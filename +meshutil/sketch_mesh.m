function sketch_mesh(vertices, faces, threshold)

if nargin < 3
  threshold = cos(30 * pi/180);
end

normals = meshutil.mesh_normals(vertices, faces);
neighbors = meshutil.mesh_neighbors(vertices, faces);
lines = {};
for j = 1:size(neighbors, 2)
  for k = 1:3
    if neighbors(k,j) > 0
      d = dot(normals(:,j), normals(:,neighbors(k,j)));
      if d < threshold
        p1 = vertices(:,faces(k, j)');
        p2 = vertices(:,faces(mod(k, 3)+1, j)');
        lines{end+1} = [p1; p2];
      end
    end
  end
end

% lines = cell2mat(lines);

figure(3);
clf;
hold on
for j = 1:length(lines)
  plot3(lines{j}([1,4]), lines{j}([2,5]), lines{j}([3,6]), 'k-')
end

      