function lines = sketch_mesh(vertices, faces, ang_threshold)
% Compute a sketch of a mesh by reducing the mesh to the edges between
% faces whose relative normal angle is greater than a given threshold (in
% radians).
% @param vertices the xyz vertex coordinates of the mesh, as a [3 x nvert] matrix
% @param faces the vertex indices of each face, as a [3 x nfaces] matrix
% @param ang_threshold the angular threshold in radians [optional]
%
% @retval lines the lines to draw, as a cell array. Each cell element is an
%         [x1 y1 z1 x2 y2 z2] vector. 
% 
% Usage: 
% >> [vertices, faces] = toolbox_graph.read_obj('/home/rdeits/drc/software/drake/examples/Glider/meshes/GliderFuselage.obj');
% >> lines = meshutil.sketch_mesh(vertices, faces, 15*pi/180);

if nargin < 3
  threshold = cos(30 * pi/180);
else
  threshold = cos(ang_threshold);
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

%figure(4)
%clf
%meshutil.plot_mesh(vertices, faces);
%axis equal
%view(40, 32)
%title('original')

%figure(3);
%clf;
hold on
for j = 1:length(lines)
  plot3(lines{j}([1,4]), lines{j}([2,5]), lines{j}([3,6]), 'k-')
end
meshutil.plot_mesh(vertices, faces, struct('patch_spec', {{[.7,.7,.7], 'FaceAlpha', 1, 'EdgeColor', 'none'}}));
%axis equal
%view(40, 32)
%title('sketch')

      