function terrain_region = bodyCSpaceRegion(planar_region, initial_yaw, X, Y, Z)

% original_foot_shape = [-0.13, -0.13, 0.13, 0.13;
%               0.0562, -0.0562, 0.0562, -0.0562];
original_foot_shape = [-0.12, -0.12, 0.11, 0.11;
              0.04, -0.04, 0.04, -0.04];

 
collision_boxes = struct('z', [0, 0.05, 0.35, 0.75, 1.15],...
  'boxes', {{[],...
     [-0.13, -0.13, 0.14, 0.14; 0.0562, -0.0562, 0.0562, -0.0562],...
     [-0.13, -0.13, 0.25, 0.25; .25, -.25, .25, -.25],...
     [-0.2, -0.2, 0.25, 0.25; .4, -.4, .4, -.4],...
     [-0.35, -0.35, 0.25, 0.25; .4, -.4, .4, -.4]}});
collision_max = 0;
for j = 1:length(collision_boxes.boxes)
  if ~isempty(collision_boxes.boxes{j})
  collision_max = max(collision_max, max(max(abs(collision_boxes.boxes{j}))));
  end
end

normal = planar_region.normal;
normal = normal/norm(normal);

R = rotmat(initial_yaw);

c_region = struct('A', [], 'b', []);

%% First, shrink the c-space region by the shape of the foot

% Rotate to the initial orientation
foot_shape = R * original_foot_shape;
for j = 1:length(collision_boxes.boxes)
  if ~isempty(collision_boxes.boxes{j})
    collision_boxes.boxes{j} = R * collision_boxes.boxes{j};
  end
end

% Reorient the foot onto the terrain plane
c = cross([0;0;1], normal);
if norm(c) >= 1e-6
  ax = c / norm(c);
  angle = asin(norm(c));
  Rplane = axis2rotmat([ax; angle]);
else
  Rplane = eye(3);
end
foot_shape =  Rplane * [foot_shape; zeros(1, size(foot_shape, 2))];
foot_shape = foot_shape(1:2,:);


c_region = shrink_by_relative_rotation(planar_region.A(:,1:2), planar_region.b, foot_shape);


%% Now, shrink the c-space further by inserting planes for upper body collisions
% compute distance from the heightmap Z to the current plane
% n' * [x;y;z] = n' * point
% z = (n'*point - n(1:2)'*[x;y])/n(3)

ell = planar_region.getXYEllipsoid();
C = ell.C;
Cinv = inv(C);
Cinv2 = Cinv * Cinv';
d = ell.d;

dZ = Z - reshape((planar_region.normal'*planar_region.point - planar_region.normal(1:2)'*[reshape(X,1,[]);reshape(Y,1,[])])/planar_region.normal(3), size(Z));
xypoly = iris.Polytope(planar_region.A(:,1:2), planar_region.b).normalize();
dists = bsxfun(@minus, xypoly.A * [reshape(X,1,[]); reshape(Y,1,[])], xypoly.b);
dists = max(dists, [], 1);
dists_mask = reshape(dists, size(Z)) <= collision_max;

for j = 1:length(collision_boxes.z)
  if isempty(collision_boxes.boxes{j})
    continue
  end
  zmin = collision_boxes.z(j);
  if j < length(collision_boxes.z)
    zmax = collision_boxes.z(j+1);
  else
    zmax = inf;
  end
  z_range_mask = dZ >= zmin & dZ <= zmax;
  mask = z_range_mask & dists_mask;
%   mask = z_range_mask;
  A = (2 * Cinv2 * bsxfun(@minus, [reshape(X(mask),1,[]); reshape(Y(mask),1,[])], d))';
  b = sum(A' .* [reshape(X(mask),1,[]); reshape(Y(mask),1,[])], 1)';
%   lcmgl = LCMGLClient('z_range_polytope');
%   iris.TerrainRegion(A, b, [], [], planar_region.point, planar_region.normal).getXYZPolytope().drawLCMGL(lcmgl);
%   lcmgl.switchBuffers();
  body_c_region = shrink_by_relative_rotation(A, b, collision_boxes.boxes{j});
  c_region.A = [c_region.A; body_c_region.A];
  c_region.b = [c_region.b; body_c_region.b];
end


c_region.b = c_region.b + c_region.A * [0;0;initial_yaw];


% Add limits on yaw
c_region.A = [c_region.A; [0, 0, 1; 0, 0, -1]];
c_region.b = [c_region.b; [initial_yaw + pi; -(initial_yaw - pi)]];

xyyaw_poly = iris.Polytope(c_region.A, c_region.b).reduce();

vertices = xyyaw_poly.getVertices();
if ~isempty(vertices)
  terrain_region = iris.TerrainRegion(xyyaw_poly.A, xyyaw_poly.b, [],[],planar_region.point, planar_region.normal);
  
%   xs = iris.sample_convex_polytope(terrain_region.A, terrain_region.b, 100);
%   lcmgl = LCMGLClient('sampled_foot_poses');
%   lcmgl.glColor3f(.2,.9,.2);
%   for j = 1:size(xs, 2)
%     xyyaw = xs(:,j);
%     % n1 * x + n2 * y + n3 * z = n*p
%     % z = (n*p - n1x - n2y) / n3
%     z = 0.05 + (planar_region.normal' * planar_region.point - planar_region.normal(1:2)' * xyyaw(1:2)) / planar_region.normal(3); 
%     foot_shape = bsxfun(@plus, [xyyaw(1:2); z], Rplane * rpy2rotmat([0;0;xyyaw(3)]) * [original_foot_shape; zeros(1, size(original_foot_shape, 2))]);
%     lcmgl.glBegin(lcmgl.LCMGL_LINES);
%     k = convhull(foot_shape(1,:), foot_shape(2,:));
%     for i = 1:length(k)-1
%       lcmgl.glVertex3d(foot_shape(1,k(i)), foot_shape(2,k(i)), foot_shape(3,k(i)));
%       lcmgl.glVertex3d(foot_shape(1,k(i+1)), foot_shape(2,k(i+1)), foot_shape(3,k(i+1)));
%     end
%     lcmgl.glEnd();
%   end 
%   lcmgl.switchBuffers();
%   disp('here');
else
  terrain_region = []; % system has no feasible set
end

end


function new_c_region = shrink_by_relative_rotation(A, b, contact_pts)
  Rdot = [0, -1; 1, 0];
  contact_vel = Rdot * contact_pts;

  new_c_region = struct('A', [], 'b', []);

  for i = 1:size(A, 1)
    ai = A(i,:);
    n = norm(ai);
    ai = ai / n;
    bi = b(i) / n;

    p = ai * contact_pts;
    v = ai * contact_vel;

    mask = p >= 0 | v >= 0;
    for j = 1:length(mask)
      if mask(j)
        new_c_region.A(end+1,:) = [ai, v(j)];
        new_c_region.b(end+1,1) = bi - p(j);
      end
    end
  end
end


