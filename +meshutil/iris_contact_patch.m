% Compute a convex patch on the surface of a mesh. This patch wil contain
% only normals within a specified angular tolerance of the normal to the
% selected face. 

if exist('bunny_data.mat') == 2
  load('bunny_data', 'vertices', 'faces', 'normals', 'neighbors');
else
  disp('loading obj');
  tic
  [vertices, faces] = meshutil.toolbox_graph.read_mesh('bunny.obj');
  toc
  disp('computing normals');
  tic
  normals = meshutil.mesh_normals(vertices, faces);
  toc
  disp('computing neighbors');
  neighbors = meshutil.mesh_neighbors(vertices, faces);
  toc
  save('bunny_data', 'vertices', 'faces', 'normals', 'neighbors');
end

SELECTED_FACE_IDX = randi(size(faces, 2));
ANGULAR_THRESHOLD = cos(35 * pi/180);


normals = meshutil.mesh_normals(vertices, faces);
       
figure(1)
clf
meshutil.plot_mesh(vertices, faces)

seed_idx = SELECTED_FACE_IDX;
seed_pt = mean(vertices(:,faces(:,seed_idx)), 2);
seed_normal = normals(:,seed_idx);
xprime = vertices(:,faces(1,seed_idx)) - seed_pt;
xprime = xprime / norm(xprime);
yprime = cross(seed_normal, xprime);
yprime = yprime / norm(yprime);

disp('finding obstacle faces')
tic
prods = sum(bsxfun(@times, seed_normal, normals), 1);
obs_mask = prods >= 0 & prods < ANGULAR_THRESHOLD;
obs_idx = find(obs_mask);
obstacles = cell(1,length(obs_idx));
for j = 1:length(obs_idx)
  v = bsxfun(@minus, vertices(:,faces(:,obs_idx(j))'), seed_pt);
  v_dot_xprime = sum(bsxfun(@times, v, xprime), 1);
  v_dot_yprime = sum(bsxfun(@times, v, yprime), 1);
  obstacles{j} = [v_dot_xprime; v_dot_yprime];
end
toc

disp('finding obstacle edges')
tic
for j = 1:size(faces, 2)
  if obs_mask(j)
    continue
  end
  for k = 1:3
    neighbor_face = neighbors(k,j);
    % neighbor value being 0 means no neighbors along this edge
    if (neighbor_face == 0) || ...
      (normals(:,neighbor_face)' * normals(:,j) <= cos(pi/2 - ANGULAR_THRESHOLD))
      v1 = vertices(:,faces(k,j));
      v2 = vertices(:,faces(mod(k,3)+1,j));
      v = bsxfun(@minus, [v1, v2], seed_pt);
      v_dot_xprime = sum(bsxfun(@times, v, xprime), 1);
      v_dot_yprime = sum(bsxfun(@times, v, yprime), 1);
      obstacles{end+1} = [v_dot_xprime; v_dot_yprime];
    end
  end
end
toc

figure(2)
clf
for j = 1:length(obstacles)
  patch(obstacles{j}(1,:), obstacles{j}(2,:), 'k', 'FaceAlpha', 0.5)
end
bounding_box = max(max(vertices, [], 2) - min(vertices, [], 2));
lb = bounding_box * [-1;-1];
ub = bounding_box * [1;1];
A_bounds = [-diag(ones(2,1)); diag(ones(2,1))];
b_bounds = [-lb; ub];
start = [0;0];
disp('running IRIS')
tic
[A, b, C, d, results] = iris.inflate_region(obstacles, A_bounds, b_bounds, start);
toc

disp('drawing the result')
tic
iris.drawing.animate_results(results);

A_3d = A * [reshape(xprime, 1, []); reshape(yprime, 1, [])];
b_3d = b + A * [reshape(xprime, 1, []); reshape(yprime, 1, [])] * seed_pt;

Aeq = reshape(seed_normal, 1, []);
beq = dot(seed_normal, seed_pt);
figure(1)
V = iris.thirdParty.polytopes.lcon2vert(A_3d, b_3d, Aeq, beq);
iris.drawing.drawPolyFromVertices(V', 'r');
plot3(seed_pt(1), seed_pt(2), seed_pt(3), 'go', 'MarkerSize', 10, 'MarkerFacecolor', 'g');
toc