function cspace_polytope = bodyCSpaceRegion(planar_polytope, initial_yaw, X, Y, Z);

original_foot_shape = [-0.13, -0.13, 0.13, 0.13;
              0.0562, -0.0562, 0.0562, -0.0562];
 
collision_boxes = struct('z', [0, 0.05, 0.35, 0.75, 1.15],...
  'boxes', {[],...
     [-0.13, -0.13, 0.13, 0.13; 0.0562, -0.0562, 0.0562, -0.0562],...
     [-0.13, -0.13, 0.25, 0.25; .25, -.25, .25, -.25],...
     [-0.2, -0.2, 0.25, 0.25; .4, -.4, .4, -.4],...
     [-0.35, -0.35, 0.25, 0.25; .4, -.4, .4, -.4]});
   
normal = planar_polytope.Aeq';
normal = normal/norm(normal);

R = rotmat(initial_yaw);
Rdot = [0, -1; 1, 0];

c_region = struct('A', [], 'b', []);

%% First, shrink the c-space region by the shape of the foot

% Rotate to the initial orientation
foot_shape = R * original_foot_shape;

% Reorient the foot onto the terrain plane
c = cross([0;0;1], normal);
ax = c / norm(c);
angle = asin(norm(c));
Rplane = axis2rotmat([ax; angle]);
foot_shape =  Rplane * [foot_shape; zeros(1, size(foot_shape, 2))];
foot_shape = foot_shape(1:2,:);

contact_pts = foot_shape;
contact_vel = Rdot * contact_pts;

new_c_region = struct('A', [], 'b', []);
A = planar_polytope.A(:,1:2);
b = planar_polytope.b;

for j = 1:size(A, 1)
  ai = A(j,:);
  n = norm(ai);
  ai = ai / n;
  bi = b(j) / n;
  
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

new_c_region.b = new_c_region.b + new_c_region.A * [0;0;-initial_yaw];

c_region.A = [c_region.A; new_c_region.A];
c_region.b = [c_region.b; new_c_region.b];

% Add limits on yaw
c_region.A = [c_region.A; [0, 0, 1; 0, 0, -1]];
c_region.b = [c_region.b; [initial_yaw + pi; -(initial_yaw - pi)]];

cspace_polytope = iris.Polytope(c_region.A, c_region.b);

% simplify the polytope by converting to V-rep and back
vertices = cspace_polytope.getVertices();
if ~isempty(vertices)
  cspace_polytope = iris.Polytope.fromVertices(vertices);
  
  xs = iris.sample_convex_polytope(cspace_polytope.A, cspace_polytope(end).b, 200);
  lcmgl = LCMGLClient('sampled_foot_poses');
  for j = 1:size(xs, 2)
    xyyaw = xs(:,j);
    foot_shape = bsxfun(@plus, [xyyaw(1:2); 0.2], Rplane * rpy2rotmat([0;0;xyyaw(3)]) * [original_foot_shape; zeros(1, size(original_foot_shape, 2))]);
    lcmgl.glBegin(lcmgl.LCMGL_LINES);
    k = [1:size(foot_shape, 2), 1];
    for i = 1:length(k)-1
      lcmgl.glVertex3d(foot_shape(1,k(i)), foot_shape(2,k(i)), foot_shape(3,k(i)));
      lcmgl.glVertex3d(foot_shape(1,k(i+1)), foot_shape(2,k(i+1)), foot_shape(3,k(i+1)));
    end
    lcmgl.glEnd();
  end 
  lcmgl.switchBuffers();
else
  cspace_polytope = []; % system has no feasible set
end



