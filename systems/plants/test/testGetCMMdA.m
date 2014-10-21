function testGetCMMdA()
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
    nq = r.getNumPositions();
    q = rand(nq,1);
    [A,dA] = geval(@r.getCMMdA,q,struct('grad_method',{{'user','taylorvar'}}));
    [A,dA] = geval(@r.getCMMdA,q, ...
                   struct('grad_method',{{'user','numerical'}},'tol',1e-6,'diff_type','central'));
  end
end
