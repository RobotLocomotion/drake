function testcentroidalMomentumMatrix()
test_cases = struct('urdf', ...
  {[getDrakePath, '/examples/Atlas/urdf/atlas_minimal_contact.urdf'], ...
  [getDrakePath, '/systems/plants/test/FallingBrick.urdf'], ...
  [getDrakePath, '/examples/Acrobot/Acrobot.urdf'], ...
  [getDrakePath, '/examples/Pendulum/Pendulum.urdf']}, ...
  'floating',{true,true,false,false});
for test_case = test_cases
  options.floating = test_case.floating;
  %options.floating = false;
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:NonPSDInertia');
  warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
  r = RigidBodyManipulator(test_case.urdf,options);
  warning(w);
  checkGradients(r);
  checkMex(r);
end
end

function checkGradients(r)
nq = r.getNumPositions();
q = rand(nq,1);
[~,~] = geval(@r.centroidalMomentumMatrix,q,struct('grad_method',{{'user','taylorvar'}}));
[~,~] = geval(@r.centroidalMomentumMatrix,q, ...
  struct('grad_method',{{'user','numerical'}}, ...
  'tol',1e-6,'diff_type','central','da',1e-6));
end

function checkMex(r)
nq = r.getNumPositions();
rng(1, 'twister');
q = rand(nq,1);

kinsol_options.use_mex = false;
kinsol_options.compute_gradients = true;
kinsol = r.doKinematics(q, [], kinsol_options);
[A, dA] = r.centroidalMomentumMatrix(kinsol);

kinsol_options.use_mex = true;
kinsol = r.doKinematics(q, [], kinsol_options);
[A_mex, dA_mex] = r.centroidalMomentumMatrix(kinsol);
valuecheck(A_mex, A);
valuecheck(dA_mex, dA);
end
