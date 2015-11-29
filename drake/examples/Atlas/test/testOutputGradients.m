function testOutputGradients()
%% Load the model with a floating base
atlas_path = fullfile(getDrakePath,'examples','Atlas');
path_handle = addpathTemporary(atlas_path);
dt = .001;
options.floating = true;
options.dt = dt;
options.terrain = RigidBodyFlatTerrain;
r = Atlas(fullfile(atlas_path, 'urdf', 'atlas_convex_hull.urdf'), options);
%r = r.removeCollisionGroupsExcept({'heel','toe','back','front','knee','butt'});
r = compile(r);
nu = r.getNumInputs();

%% Step through dynamics
x0 = Point(r.getStateFrame());
x0 = resolveConstraints(r,x0);

u = zeros(nu,1);
x = x0.double();
t = 0;

geval_options.grad_method = {'user', 'numerical'};
geval_options.tol = 1e-5;
[~, ~] = geval(1, @(t, x, u) r.update(t, x, u), t, x, u, geval_options);

end