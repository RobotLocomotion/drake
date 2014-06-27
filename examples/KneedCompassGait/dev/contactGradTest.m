function [f,df,phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = contactGradTest
  options.terrain = RigidBodyFlatTerrain();
  options.floating = true;
  options.ignore_self_collisions = true;
  p = PlanarRigidBodyManipulator('../KneedCompassGait.urdf',options);
  q = [.2;1+1e-6;zeros(4,1)];
  geval_opt.grad_method = {'numerical'};
  geval_opt.tol = 1e-8;
  [f,df] = geval(@testFun,q,geval_opt);
  [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = p.contactConstraints(q,false,struct('terrain_only',false));
  
  function [f,df] = testFun(q)
    [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = p.contactConstraints(q,false,struct('terrain_only',false));
    f = n;
    df = dn;
  end
  
end